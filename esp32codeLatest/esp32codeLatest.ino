// Sensor 1 — creates its own WiFi network (SoftAP) and broadcasts distance data over UDP

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


//---- SoftAP config ----
const char* AP_SSID     = "MoleTracker";
const char* AP_PASSWORD = "12345678";
const int   GAME_PORT   = 4212;         // must match LISTEN_PORT in Java

// Broadcast address on the AP's subnet (192.168.4.x is ESP32's default AP range)
IPAddress broadcastIP(192, 168, 4, 255);

//---- Rate limiting ----
const unsigned long SEND_INTERVAL_MS = 50;
unsigned long lastSendTime = 0;

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

WiFiUDP udp;
Servo motor1;

enum State { PLAYING, IDLE };
State currentState = PLAYING;

unsigned long lastLedToggle = 0;
bool ledState = false;
const unsigned long LED_INTERVAL = 500;

// Stores previous distance to prevent "zero" readings
float prevDistance = 0;

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

void loop(void) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  float distance = duration * 0.034 / 2;

  // Remove any "zero" readings for consistent measuring
  if (duration != 0) {
    prevDistance = distance;
  } else {
    distance = prevDistance;
  } 

  Serial.print("Distance: ");
  Serial.println(distance); 

  myData.distance = distance;

  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  Serial.print("Sensor ");
  Serial.print(IS_SENSOR_1 ? "1 (this board): " : "2 (this board): ");
  Serial.print("distance=");
  Serial.println(myData.distance);

  if (peerDataReceived) {
    Serial.print("Peer sensor: distance=");
    Serial.println(peerData.distance);

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

    Serial.print("d1=");
    Serial.print(distance1);
    Serial.print(" d2=");
    Serial.println(distance2);

    // ---- Triangulation ----
    TriangulationResult pos = computeTriangulation(distance1, distance2);

    if (pos.valid) {
      Serial.print("angle1=");
      Serial.print(pos.angle1Deg);
      Serial.print(" angle2=");
      Serial.print(pos.angle2Deg);
      Serial.print(" x=");
      Serial.print(pos.x);
      Serial.print(" y=");
      Serial.println(pos.y);

      // Point this board's servo using the angle measured from its own sensor
      float myAngleDeg = IS_SENSOR_1 ? pos.angle1Deg : pos.angle2Deg;
      motor1.write((int)constrain(myAngleDeg, 0, 180));
    } else {
      Serial.println("Invalid triangle - holding servo at center");
      motor1.write(CENTER_SERVO_ANGLE);
    }

    unsigned long now = millis();
    if (IS_SENSOR_1 && now - lastSendTime >= SEND_INTERVAL_MS) {
      sendDistances(distance1, distance2);
      lastSendTime = now;
    }
  } else {
    Serial.println("Waiting for peer data...");
  }

  Serial.println("---");

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

  delay(500);
}

void sendDistances(float d1, float d2) {
  String payload = "{\"d1\":" + String(d1, 2) + ",\"d2\":" + String(d2, 2) + "}";

  udp.beginPacket(broadcastIP, GAME_PORT);
  udp.print(payload);
  udp.endPacket();

  Serial.println("Sent: " + payload);
}
