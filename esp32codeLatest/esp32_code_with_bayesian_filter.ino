// Sensor 1 — creates its own WiFi network (SoftAP) and broadcasts distance data over UDP
//
// Changes from the previous version:
//  - The blocking delay(500) is gone. loop() now schedules exactly 16 readings per second
//    (one every 62.5 milliseconds) using micros(), so the board never sits frozen.
//  - Every raw reading goes through a Kalman filter: a Bayesian "best guess" filter that
//    combines "where the player should be, given how they were moving" with "what the sensor
//    says", and throws away readings that would need the player to move impossibly fast.

#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>
#include <math.h>

#define ECHO_PIN 26
#define TRIG_PIN 27
#define Servo_PWM 25
#define ledPin 2

#define IS_SENSOR_1 true

// true  = print only the raw and filtered distance, in the format the Arduino IDE's
//         Serial Plotter understands (Tools > Serial Plotter), so you can watch the filter work.
// false = normal text logging.
#define SERIAL_PLOTTER_MODE false


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

//---- Triangulation ----
const float BASELINE_CM = 75.0; // fixed distance between Sensor 1 and Sensor 2
const int   CENTER_SERVO_ANGLE = 90; // where the servo parks when there's no valid triangle

struct TriangulationResult {
  bool valid;
  float angle1Deg; // angle at Sensor 1, measured from the baseline
  float angle2Deg; // angle at Sensor 2, measured from the baseline
  float x;         // target x position (cm), origin at Sensor 1
  float y;         // target y position (cm), 0 = baseline, positive = into the field
};

// Tuning knobs. These trade smoothness against responsiveness.

const float MAX_PLAYER_SPEED_CENTIMETRES_PER_SECOND = 200.0; // 2 metres per second = 12.5 centimetres per reading
const float SENSOR_NOISE_CENTIMETRES = 3.0;    // typical wobble of a good reading (standard deviation). Bigger = smoother but laggier
const float PLAYER_ACCELERATION_NOISE = 300.0; // how sharply a player changes speed (centimetres per second, per second). Bigger = quicker to react but jumpier
const float NOISE_ALLOWANCE_CENTIMETRES = 3.0 * SENSOR_NOISE_CENTIMETRES; // slack added on top of the speed limit
const int   MAX_CONSECUTIVE_REJECTIONS = 8;    // half a second of "impossible" readings in a row: assume the filter lost track and start again
const float VELOCITY_DECAY_WITHOUT_READING = 0.8; // with no usable reading, assume the player is slowing down rather than gliding on forever
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
  TriangulationResult result = {false, 0, 0, 0, 0};

  if (d1 <= 0 || d2 <= 0) return result; // no valid reading from one or both sensors

  // Triangle inequality — these three side lengths must be able to form a real triangle.
  // If sensor noise produces distances that geometrically can't both reach the same
  // point given the fixed baseline, bail out rather than feeding acos() a bad value.
  if (d1 + d2 < baseline) return result;
  if (fabs(d1 - d2) > baseline) return result;

  // Law of cosines, solved for the angle at each sensor
  float cosTheta1 = (d1 * d1 + baseline * baseline - d2 * d2) / (2 * d1 * baseline);
  cosTheta1 = constrain(cosTheta1, -1.0, 1.0); // guard against float rounding pushing just past +-1

  float cosTheta2 = (d2 * d2 + baseline * baseline - d1 * d1) / (2 * d2 * baseline);
  cosTheta2 = constrain(cosTheta2, -1.0, 1.0);

  float theta1 = acos(cosTheta1);
  float theta2 = acos(cosTheta2);

  result.valid = true;
  result.angle1Deg = theta1 * 180.0 / PI;
  result.angle2Deg = 180 - (theta2 * 180.0 / PI);
  result.x = d1 * cos(theta1);
  result.y = d1 * sin(theta1);

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

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&peerData, incomingData, sizeof(peerData));
  peerDataReceived = true;
}

void setup(void) {
  Serial.begin(115200);

  pinMode(ledPin, OUTPUT);
  motor1.attach(Servo_PWM);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  lastReadingTime = micros();
  nextReadingTime = lastReadingTime;

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
    Serial.print(IS_SENSOR_1 ? "1" : "2");
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

  if (peerDataReceived) {
    float distance1, distance2;
    if (IS_SENSOR_1) {
      if (millis() - lastLedToggle >= LED_INTERVAL) {
        ledState = !ledState;
        digitalWrite(ledPin, ledState);
        lastLedToggle = millis();
      }
      distance1 = myData.distance;
      distance2 = peerData.distance;
    } else {
      distance1 = peerData.distance;
      distance2 = myData.distance;
    }

    // ---- Triangulation ----
    TriangulationResult pos = computeTriangulation(distance1, distance2);

    if (pos.valid) {
      if (!SERIAL_PLOTTER_MODE) {
        Serial.print("d1=");
        Serial.print(distance1);
        Serial.print(" d2=");
        Serial.print(distance2);
        Serial.print(" angle1=");
        Serial.print(pos.angle1Deg);
        Serial.print(" angle2=");
        Serial.print(pos.angle2Deg);
        Serial.print(" x=");
        Serial.print(pos.x);
        Serial.print(" y=");
        Serial.println(pos.y);
      }

      // Point this board's servo using the angle measured from its own sensor
      float myAngleDeg = IS_SENSOR_1 ? pos.angle1Deg : pos.angle2Deg;
      motor1.write((int)constrain(myAngleDeg, 0, 180));
    } else {
      if (!SERIAL_PLOTTER_MODE) Serial.println("Invalid triangle - holding servo at center");
      motor1.write(CENTER_SERVO_ANGLE);
    }

    unsigned long nowMs = millis();
    if (IS_SENSOR_1 && nowMs - lastSendTime >= SEND_INTERVAL_MS) {
      sendDistances(distance1, distance2);
      lastSendTime = nowMs;
    }
  } else if (!SERIAL_PLOTTER_MODE) {
    Serial.println("Waiting for peer data...");
  }

  // if (duration != 0 && distance <= 10) {
  //   Serial.println("ALARM");
  //   motor1.write(90);
  // } else {
  //   Serial.println("ALARM_OFF");
  //   motor1.write(0);
  // }

  switch (currentState) {
    case IDLE:
      break;
    case PLAYING:
      break;
  }
}

void sendDistances(float d1, float d2) {
  String payload = "{\"d1\":" + String(d1, 2) + ",\"d2\":" + String(d2, 2) + "}";

  udp.beginPacket(broadcastIP, GAME_PORT);
  udp.print(payload);
  udp.endPacket();

  if (!SERIAL_PLOTTER_MODE) Serial.println("Sent: " + payload);
}
