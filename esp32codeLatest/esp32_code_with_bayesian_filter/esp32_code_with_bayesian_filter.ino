// Sensor board — creates its own WiFi network (SoftAP) and broadcasts player position over UDP
//
// Changes in this version (fixes for blind spots and jumpiness):
//  1. Triangle check corrected (it was inverted) and made tolerant: small violations are clamped
//     onto the baseline instead of being thrown away; only gross violations are rejected.
//  2. Body-radius compensation: the sensors measure to the nearest surface of the player, so half
//     the player's width is added to each distance before triangulating (centre-based geometry).
//  3. Servos no longer snap to centre on an invalid frame. They hold the last angle, and after a
//     timeout do a slow search sweep. All servo commands are rate-limited with a small deadband.
//  4. The two boards take turns firing. The follower board phase-locks itself half a reading
//     interval after each packet from the leader, so the sensors never ping at the same moment
//     (no cross-talk). Peer data older than PEER_STALE_MS is ignored.
//  5. Flat-triangle smoothing: the final x,y is smoothed harder when the triangle is nearly flat
//     (player close to the line between the sensors or its extensions) and step-limited.
//  6. Invalid frames no longer send (0, 0) to the game; nothing is sent, so the game keeps the
//     last position.

#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>
#include <math.h>

#define ECHO_PIN 26
#define TRIG_PIN 27
#define Servo_PWM 25
#define ledPin 2
#define buzzerPin 32

#define IS_SENSOR_right false

// true  = print only the raw and filtered distance, in the format the Arduino IDE's
//         Serial Plotter understands (Tools > Serial Plotter), so you can watch the filter work.
// false = normal text logging.
#define SERIAL_PLOTTER_MODE true


//---- SoftAP config ----
const char* AP_SSID     = "MoleTracker";
const char* AP_PASSWORD = "12345678";
const int   GAME_PORT   = 4212;         // must match LISTEN_PORT in Java

// Broadcast address on the AP's subnet (192.168.4.x is ESP32's default AP range)
IPAddress broadcastIP(192, 168, 4, 255);

//---- Rate limiting ----
const unsigned long SEND_INTERVAL_MS = 50;
unsigned long lastSendTime = 0;

//---- Reading schedule: 16 readings per second ----
const unsigned long READINGS_PER_SECOND = 16;
const unsigned long READING_INTERVAL_MICROSECONDS = 1000000UL / READINGS_PER_SECOND; // 62500 = 62.5 milliseconds
const unsigned long ECHO_TIMEOUT_MICROSECONDS = 30000; // longest wait for an echo (target about 5 metres away)
unsigned long nextReadingTime = 0; // micros() value when the next reading is due
unsigned long lastReadingTime = 0; // micros() value of the previous reading

//---- Board-to-board sync ----
// The LEADER board (IS_SENSOR_right = true) free-runs. The FOLLOWER board re-times itself every
// time a packet from the leader arrives so that it fires half an interval later.
const bool IS_STAGGER_FOLLOWER = !IS_SENSOR_right;
const unsigned long STAGGER_OFFSET_MICROSECONDS = READING_INTERVAL_MICROSECONDS / 2; // 31.25 ms
const unsigned long PEER_STALE_MS = 150; // ignore the other board's distance if it is older than this

//---- Triangulation ----
const float BASELINE_CM = 75.0; // fixed distance between Sensor 1 and Sensor 2
const int   CENTER_SERVO_ANGLE = 90; // where the servo starts

struct TriangulationResult {
  bool valid;
  float angle1Deg; // angle at Sensor 1, measured from the baseline
  float angle2Deg; // angle at Sensor 2, measured from the baseline (mirrored for the servo)
  float x;         // target x position (cm)
  float y;         // target y position (cm), 0 = baseline, positive = into the field
  float quality;   // 0 = triangle is flat (unreliable position), 1 = well-shaped triangle
};

//---- Player body model ----
const float BODY_RADIUS_CM = 12.5;        // half the ~25 cm player width. Calibrate with tape-measured marks.
const float TRIANGLE_TOLERANCE_CM = 10.0; // slack on the triangle checks (sensor noise + oval shape). Bigger = fewer rejections
const float FIELD_SIZE_CM = 150.0;        // play area is 150 x 150 cm

//---- Servo behaviour ----
const float SERVO_MAX_STEP_DEG = 10.0;     // most the servo may move per reading (about 160 degrees per second)
const float SERVO_DEADBAND_DEG = 1.5;      // ignore tiny angle changes so the servo doesn't buzz
const bool  ENABLE_SEARCH_SWEEP = true;    // set false to just hold the last angle when the player is lost
const unsigned long SEARCH_TIMEOUT_MS = 1500; // how long to hold before starting to search
const float SEARCH_STEP_DEG = 3.0;         // sweep speed per reading
const float SEARCH_MIN_DEG = 15.0;
const float SEARCH_MAX_DEG = 165.0;

//---- Output smoothing (applied to the final x, y) ----
const float SMOOTH_ALPHA_GOOD = 0.7;       // responsiveness when the triangle is well shaped (0..1, higher = faster)
const float SMOOTH_ALPHA_FLAT = 0.2;       // responsiveness when the triangle is flat (heavier smoothing)
const float STEP_LIMIT_HEADROOM = 1.5;     // output may move at most 1.5x top player speed per reading
const unsigned long LOST_RESET_MS = 500;   // lost this long: forget the smoothed position and jump to the next one

// Tuning knobs. These trade smoothness against responsiveness.

const float MAX_PLAYER_SPEED_CENTIMETRES_PER_SECOND = 200.0; // 2 metres per second = 12.5 centimetres per reading
const float SENSOR_NOISE_CENTIMETRES = 3.0;    // typical wobble of a good reading (standard deviation). Bigger = smoother but laggier
const float PLAYER_ACCELERATION_NOISE = 200.0; // how sharply a player changes speed (centimetres per second, per second). Bigger = quicker to react but jumpier
const float NOISE_ALLOWANCE_CENTIMETRES = 2.0 * SENSOR_NOISE_CENTIMETRES; // slack added on top of the speed limit
const int   MAX_CONSECUTIVE_REJECTIONS = 8;    // half a second of "impossible" readings in a row: assume the filter lost track and start again
const float VELOCITY_DECAY_WITHOUT_READING = 0.4; // with no usable reading, assume the player is slowing down rather than gliding on forever
const float MIN_SENSOR_RANGE_CENTIMETRES = 2.0;
const float MAX_SENSOR_RANGE_CENTIMETRES = 200.0;

struct DistanceFilter {
  bool  initialised;
  float position;  // best estimate of the distance to the player (centimetres)
  float velocity;  // best estimate of how fast that distance is changing (centimetres per second, negative = approaching)
  float positionVariance;
  float positionVelocityCovariance;
  float velocityVariance;
  float secondsSinceAcceptedReading;
  int   consecutiveRejections;
};

TriangulationResult computeTriangulation(float d1, float d2, float baseline = BASELINE_CM) {
  TriangulationResult result = {false, 0, 0, 0, 0, 0};

  if (d1 <= 0 || d2 <= 0) return result; // no valid reading from one or both sensors

  // The sensors measure to the nearest surface of the player, not their centre. Add the body
  // radius so the distances describe the player's centre; this restores d1 + d2 >= baseline for
  // players standing near the line between the sensors.
  d1 += BODY_RADIUS_CM;
  d2 += BODY_RADIUS_CM;

  // Triangle inequality with tolerance. Small violations (noise, oval body shape) are allowed and
  // are handled by the clamping below, which places the target on the baseline. Only gross
  // violations, which are real glitches, are rejected.
  if (d1 + d2 < baseline - TRIANGLE_TOLERANCE_CM) return result;
  if (fabs(d1 - d2) > baseline + TRIANGLE_TOLERANCE_CM) return result;

  // Law of cosines, solved for the angle at each sensor
  float cosTheta1 = (d1 * d1 + baseline * baseline - d2 * d2) / (2 * d1 * baseline);
  cosTheta1 = constrain(cosTheta1, -1.0, 1.0); // handles tolerated violations and float rounding

  float cosTheta2 = (d2 * d2 + baseline * baseline - d1 * d1) / (2 * d2 * baseline);
  cosTheta2 = constrain(cosTheta2, -1.0, 1.0);

  float theta1 = acos(cosTheta1);
  float theta2 = acos(cosTheta2);

  // Angle at the player is what is left of the triangle's 180 degrees. When it is close to
  // 0 or 180 degrees the triangle is flat and a small distance error becomes a big position error.
  float gamma = PI - theta1 - theta2;
  float quality = sin(gamma);
  quality = constrain(quality, 0.0, 1.0);

  float x = d1 * cos(theta1) + 37.5;
  float y = d1 * sin(theta1);

  result.valid = true;
  result.angle1Deg = theta1 * 180.0 / PI;
  result.angle2Deg = 180 - (theta2 * 180.0 / PI);
  result.x = constrain(x, 0.0, FIELD_SIZE_CM);
  result.y = constrain(y, 0.0, FIELD_SIZE_CM);
  result.quality = quality;

  return result;
}


//---- Distance filter (Kalman filter with a player speed limit) ----

DistanceFilter myFilter = {}; // starts uninitialised; the first good reading sets it up

void resetFilter(DistanceFilter &f, float measurement) {
  f.initialised = true;
  f.position = measurement;
  f.velocity = 0;
  f.positionVariance = SENSOR_NOISE_CENTIMETRES * SENSOR_NOISE_CENTIMETRES;
  f.positionVelocityCovariance = 0;
  f.velocityVariance = MAX_PLAYER_SPEED_CENTIMETRES_PER_SECOND * MAX_PLAYER_SPEED_CENTIMETRES_PER_SECOND; // no idea of speed yet
  f.secondsSinceAcceptedReading = 0;
  f.consecutiveRejections = 0;
}

// Feeds one reading into the filter. Pass gotReading = false when the sensor heard no echo.
// Returns true if the reading was used, false if it was missing or rejected as impossible.
bool updateFilter(DistanceFilter &f, float measurement, bool gotReading, float elapsedSeconds) {
  if (!f.initialised) {
    if (gotReading) resetFilter(f, measurement);
    return gotReading;
  }

  float dt = elapsedSeconds;
  float dtSquared = dt * dt;
  float q = PLAYER_ACCELERATION_NOISE * PLAYER_ACCELERATION_NOISE;

  // 1. PREDICT (the "prior"): if the player kept moving at the same speed, locate the player.
  //    Uncertainty grows, because in that time they may have sped up, slowed down or turned.
  f.position += f.velocity * dt;
  f.positionVariance += 2.0 * dt * f.positionVelocityCovariance + dtSquared * f.velocityVariance + q * dtSquared * dtSquared / 4.0;
  f.positionVelocityCovariance += dt * f.velocityVariance + q * dtSquared * dt / 2.0;
  f.velocityVariance += q * dtSquared;
  f.secondsSinceAcceptedReading += dt;

  bool used = false;

  if (gotReading) {
    // 2. SANITY CHECK: The player can't have covered more than top speed x time since the
    //    last good reading, plus a little slack for sensor noise.
    float surprise = measurement - f.position;
    float maxPlausibleJump = MAX_PLAYER_SPEED_CENTIMETRES_PER_SECOND * f.secondsSinceAcceptedReading
                           + NOISE_ALLOWANCE_CENTIMETRES;

    if (fabs(surprise) <= maxPlausibleJump) {
      // 3. UPDATE (the "posterior"): blend prediction and reading, weighted by how much we trust
      //    each. That weight is the "Kalman gain": near 1 = believe the sensor, near 0 = believe
      //    the prediction. It is worked out automatically from the variances.
      float totalVariance = f.positionVariance + SENSOR_NOISE_CENTIMETRES * SENSOR_NOISE_CENTIMETRES;
      float positionGain = f.positionVariance / totalVariance;
      float velocityGain = f.positionVelocityCovariance / totalVariance;

      f.position += positionGain * surprise;
      f.velocity += velocityGain * surprise;

      float oldPositionVariance = f.positionVariance;
      float oldCovariance = f.positionVelocityCovariance;
      f.positionVariance = (1.0 - positionGain) * oldPositionVariance;
      f.positionVelocityCovariance = (1.0 - positionGain) * oldCovariance;
      f.velocityVariance -= velocityGain * oldCovariance;

      f.secondsSinceAcceptedReading = 0;
      f.consecutiveRejections = 0;
      used = true;
    } else {
      f.consecutiveRejections++;
      if (f.consecutiveRejections >= MAX_CONSECUTIVE_REJECTIONS) {
        // The sensor keeps disagreeing for half a second. At that point it is more likely the
        // filter is wrong (the player really did move, or it locked onto a bad value), so
        // trust the sensor and start over from here.
        resetFilter(f, measurement);
        used = true;
      }
    }
  }

  if (!used) {
    f.velocity *= VELOCITY_DECAY_WITHOUT_READING;
  }

  f.velocity = constrain(f.velocity, -MAX_PLAYER_SPEED_CENTIMETRES_PER_SECOND, MAX_PLAYER_SPEED_CENTIMETRES_PER_SECOND);
  f.position = constrain(f.position, MIN_SENSOR_RANGE_CENTIMETRES, MAX_SENSOR_RANGE_CENTIMETRES);
  return used;
}


WiFiUDP udp;
Servo motor1;

enum State { PLAYING, IDLE };
State currentState = PLAYING;

unsigned long lastLedToggle = 0;
bool ledState = false;
const unsigned long LED_INTERVAL = 500;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // ESP-NOW peer

typedef struct SensorData {
  int angle;
  float distance;
} SensorData;

SensorData myData;
SensorData peerData;
bool peerDataReceived = false;
esp_now_peer_info_t peerInfo;

// Set by the ESP-NOW callback, read by loop()
volatile unsigned long peerPacketMicros = 0;   // micros() when the peer's last packet arrived
volatile unsigned long lastPeerReceiveMs = 0;  // millis() when the peer's last packet arrived
volatile bool newPeerPacket = false;           // true until the follower has re-timed itself

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != (int)sizeof(peerData)) return; // ignore anything that isn't our packet
  memcpy(&peerData, incomingData, sizeof(peerData));
  peerPacketMicros = micros();
  lastPeerReceiveMs = millis();
  peerDataReceived = true;
  newPeerPacket = true;
}


//---- Servo control: hold, search, rate-limit ----
float servoAngle = CENTER_SERVO_ANGLE;  // where the servo currently is (as commanded)
int   searchDirection = 1;              // +1 or -1 while sweeping
unsigned long lastValidTriangleMs = 0;  // last time the triangulation was valid

void updateServo(bool validTriangle, float desiredAngle) {
  unsigned long nowMs = millis();
  float target = servoAngle; // default: stay where we are

  if (validTriangle) {
    lastValidTriangleMs = nowMs;
    target = constrain(desiredAngle, 0.0, 180.0);
  } else if (ENABLE_SEARCH_SWEEP && (nowMs - lastValidTriangleMs) > SEARCH_TIMEOUT_MS) {
    // Player lost for a while: sweep slowly back and forth until a valid triangle appears
    float next = servoAngle + searchDirection * SEARCH_STEP_DEG;
    if (next >= SEARCH_MAX_DEG) {
      next = SEARCH_MAX_DEG;
      searchDirection = -1;
    } else if (next <= SEARCH_MIN_DEG) {
      next = SEARCH_MIN_DEG;
      searchDirection = 1;
    }
    servoAngle = next;
    motor1.write((int)(servoAngle + 0.5));
    return;
  }

  float difference = target - servoAngle;
  if (fabs(difference) < SERVO_DEADBAND_DEG) return;

  servoAngle += constrain(difference, -SERVO_MAX_STEP_DEG, SERVO_MAX_STEP_DEG);
  motor1.write((int)(servoAngle + 0.5));
}


//---- Output smoothing for the final x, y ----
float smoothX = 0;
float smoothY = 0;
bool  smoothInitialised = false;

void smoothPosition(const TriangulationResult &pos, float elapsedSeconds, float &outX, float &outY) {
  if (!smoothInitialised) {
    smoothX = pos.x;
    smoothY = pos.y;
    smoothInitialised = true;
  } else {
    // Flat triangle = unreliable position = smooth harder
    float alpha = SMOOTH_ALPHA_FLAT + (SMOOTH_ALPHA_GOOD - SMOOTH_ALPHA_FLAT) * pos.quality;
    float maxStep = MAX_PLAYER_SPEED_CENTIMETRES_PER_SECOND * elapsedSeconds * STEP_LIMIT_HEADROOM;

    smoothX += constrain(alpha * (pos.x - smoothX), -maxStep, maxStep);
    smoothY += constrain(alpha * (pos.y - smoothY), -maxStep, maxStep);
  }
  outX = smoothX;
  outY = smoothY;
}


void setup(void) {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  motor1.attach(Servo_PWM);
  motor1.write(CENTER_SERVO_ANGLE);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lastReadingTime = micros();
  nextReadingTime = lastReadingTime;
  lastValidTriangleMs = millis(); // gives a grace period at boot before any search sweep

  // AP_STA mode: broadcasts its own network (AP) while also using WiFi radio for ESP-NOW (STA)
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.print("Access Point started. Connect your laptop to: ");
  Serial.println(AP_SSID);
  Serial.print("ESP32 AP IP: ");
  Serial.println(WiFi.softAPIP());

  udp.begin(GAME_PORT);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  myData.angle = 90; // unused in this demo, kept for struct compatibility

}

// Fires the ultrasonic sensor once. Sets gotReading to false if there was no echo, or the
// echo came back from outside the range the sensor can measure reliably.
float readRawDistance(bool &gotReading) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_MICROSECONDS);
  float distance = duration * 0.034 / 2;

  gotReading = duration != 0
            && distance >= MIN_SENSOR_RANGE_CENTIMETRES
            && distance <= MAX_SENSOR_RANGE_CENTIMETRES;
  return distance;
}

void loop(void) {
  unsigned long now = micros();

  // Follower board: every time a packet from the leader arrives, re-time the next reading to
  // half an interval after it. Because both boards run the same interval, this keeps the two
  // sensors firing alternately instead of at the same moment.
  if (IS_STAGGER_FOLLOWER && newPeerPacket) {
    newPeerPacket = false;
    nextReadingTime = peerPacketMicros + STAGGER_OFFSET_MICROSECONDS;
  }

  // Not time for the next reading yet: rest for 1 millisecond and check again.
  // (Comparing via a signed difference keeps this correct when micros() wraps back to
  // zero, which happens roughly every 71 minutes.)
  if ((long)(now - nextReadingTime) < 0) {
    delay(1);
    return;
  }

  // Due time for the next reading = this one's due time + 62.5 milliseconds. Counting from the
  // due time, not from "now", stops small lateness from adding up into drift. If we've fallen a
  // whole interval behind (say, a burst of Serial printing), restart the schedule from now
  // instead of firing several readings back to back to catch up.
  nextReadingTime += READING_INTERVAL_MICROSECONDS;
  if ((long)(now - nextReadingTime) >= 0) {
    nextReadingTime = now + READING_INTERVAL_MICROSECONDS;
  }

  float elapsedSeconds = (now - lastReadingTime) / 1000000.0;
  lastReadingTime = now;

  takeReading(elapsedSeconds);
}

void takeReading(float elapsedSeconds) {
  bool gotReading = false;
  float rawDistance = readRawDistance(gotReading);
  bool used = updateFilter(myFilter, rawDistance, gotReading, elapsedSeconds);

  // Send the cleaned-up distance to the other board. 0 means "no estimate yet",
  // which computeTriangulation() already treats as invalid.
  myData.distance = myFilter.initialised ? myFilter.position : 0;
  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (SERIAL_PLOTTER_MODE) {
    Serial.print("raw:");
    Serial.print(rawDistance);
    Serial.print(",filtered:");
    Serial.println(myData.distance);
  } else {
    Serial.print("Sensor ");
    Serial.print(IS_SENSOR_right ? "1" : "2");
    Serial.print(" (this board): raw=");
    if (gotReading) Serial.print(rawDistance);
    else            Serial.print("none");
    Serial.print(" filtered=");
    Serial.print(myData.distance);
    Serial.print(" speed=");
    Serial.print(myFilter.velocity);
    if (!used) Serial.print(gotReading ? "  [rejected: impossible jump]" : "  [missed]");
    Serial.println();
  }

  // Only trust the other board's distance if its packet arrived recently.
  // (lastRx is read before nowMs so the age can never come out negative.)
  unsigned long lastRx = lastPeerReceiveMs;
  unsigned long nowMs = millis();
  bool peerFresh = peerDataReceived && ((long)(nowMs - lastRx) <= (long)PEER_STALE_MS);
  float peerDistance = peerFresh ? peerData.distance : 0;

  float distance1, distance2;
  if (IS_SENSOR_right) {
    // LED blinks while the link to the other board is alive
    if (peerFresh && nowMs - lastLedToggle >= LED_INTERVAL) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      lastLedToggle = nowMs;
    }
    distance1 = myData.distance;
    distance2 = peerDistance;
  } else {
    distance1 = peerDistance;
    distance2 = myData.distance;
  }



  // ---- Triangulation ----
  TriangulationResult pos = computeTriangulation(distance1, distance2);

  if (pos.y != 0 && pos.y < 10) {
    tone(buzzerPin, 2000);
  } else {
    noTone(buzzerPin);
  }

  // Servo: follows the target when valid, otherwise holds, then searches after a timeout
  float myAngleDeg = IS_SENSOR_right ? pos.angle1Deg : pos.angle2Deg;
  updateServo(pos.valid, myAngleDeg);

  if (pos.valid) {
    float outX, outY;
    smoothPosition(pos, elapsedSeconds, outX, outY);

    if (!SERIAL_PLOTTER_MODE) {
      Serial.print("d1=");
      Serial.print(distance1);
      Serial.print(" d2=");
      Serial.print(distance2);
      Serial.print(" angle1=");
      Serial.print(pos.angle1Deg);
      Serial.print(" angle2=");
      Serial.print(pos.angle2Deg);
      Serial.print(" quality=");
      Serial.print(pos.quality);
      Serial.print(" x=");
      Serial.print(outX);
      Serial.print(" y=");
      Serial.println(outY);
    }

    if (IS_SENSOR_right && nowMs - lastSendTime >= SEND_INTERVAL_MS) {
      sendDistances(outX, outY);
      lastSendTime = nowMs;
    }
  } else {
    if (!SERIAL_PLOTTER_MODE) {
      Serial.println(peerFresh ? "Invalid triangle - servo holding/searching, nothing sent"
                               : "Waiting for fresh peer data...");
    }
    // Lost for a while: forget the old smoothed position so the next one isn't dragged toward it
    if (nowMs - lastValidTriangleMs > LOST_RESET_MS) {
      smoothInitialised = false;
    }
  }

  switch (currentState) {
    case IDLE:
      break;
    case PLAYING:
      break;
  }
}

void sendDistances(float x, float y) {
  String payload = "{\"d1\":" + String(x, 2) + ",\"d2\":" + String(y, 2) + "}";

  udp.beginPacket(broadcastIP, GAME_PORT);
  udp.print(payload);
  udp.endPacket();

  if (!SERIAL_PLOTTER_MODE) Serial.println("Sent: " + payload);
}
