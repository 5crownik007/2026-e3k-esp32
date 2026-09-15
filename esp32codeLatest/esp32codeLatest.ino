// Sensor 1 — creates its own WiFi network (SoftAP) and broadcasts distance data over UDP

#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <WiFiUdp.h>

#define ECHO_PIN 26
#define TRIG_PIN 27
#define Servo_PWM 25
#define ledPin 2
#define ALARM_PIN 33 // Change pin if required

#define IS_SENSOR_1 false

// SoftAP config
const char* AP_SSID     = "E_3_K_2_0_2_6";
const char* AP_PASSWORD = "whackamole123";
const int   GAME_PORT   = 4212;         // must match LISTEN_PORT in the Java game file

// Broadcast address on the AP's subnet 
IPAddress broadcastIP(192, 168, 4, 255);

// Rate limiting
const unsigned long SEND_INTERVAL_MS = 50;
unsigned long lastSendTime = 0;

WiFiUDP udp;
Servo motor1;

enum State { PLAYING, IDLE };
State currentState = PLAYING;

unsigned long lastLedToggle = 0;
bool ledState = false;
const unsigned long LED_INTERVAL = 500;

// Counts how close in cm.
const float ALARM_DISTANCE_CM = 10.0;

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
  pinMode(ALARM_PIN, OUTPUT);
  digitalWrite(ALARM_PIN, LOW); 
  motor1.attach(Servo_PWM);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // For broadcasting its own network
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
  myData.distance = distance;

  if (duration == 0) {
    Serial.println("Out of range");
  } else {
    Serial.print("Distance: ");
    Serial.println(distance);
  }

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

    unsigned long now = millis();
    if (IS_SENSOR_1 && now - lastSendTime >= SEND_INTERVAL_MS) {
      sendDistances(distance1, distance2);
      lastSendTime = now;
    }
  } else {
    Serial.println("Waiting for peer data...");
  }

  Serial.println("---");

  // Triggers alarm. Transistor turns on when the sensor is within
  // ALARM_DISTANCE_CM range. Duration != 0 is there so alarm
  // doesn't read as "very close" accidentally.

  bool tooClose = (duration != 0 && distance <= ALARM_DISTANCE_CM);
  digitalWrite(ALARM_PIN, tooClose ? HIGH : LOW);

  if (duration != 0 && distance <= 10) {
    Serial.println("ALARM");
  } else {
    Serial.println("ALARM_OFF");
  }

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
