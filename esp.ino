//#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>


const char* AP_SSID = "E_3_K_2_0_2_6";
const char* AP_PASSWORD = "whackamole123"; // must be 8+ characters

// ---------- Java game app settings ----------
// Once your laptop joins the AP_SSID network above, find your
// laptop's IP address on THAT network (ipconfig/ifconfig) and put
// it here - it will likely be something like 192.168.4.2.
const char* GAME_IP = "192.168.4.2"; //Laptops IP
const unsigned int GAME_PORT = 4210;

WiFiUDP udp;

#define ECHO_PIN 26
#define TRIG_PIN 27

#define I2C_ADDR 0x27
#define LCD_SDA 21
#define LCD_SCL 22
#define Servo_PWM 18

//LiquidCrystal_I2C lcd (0x27,LCD_SDA,LCD_SCL);
//Servo motor1;

enum State {
  PLAYING,
  IDLE
};

State currentState = PLAYING;

float readDistanceCM();
void sendPosition(float x, float y);

void setup(void) {
  //motor1.attach(Servo_PWM);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // lcd.init();
  // lcd.backlight();
  // lcd.setCursor(3, 0);
  // lcd.print("Hello, world!");
  // lcd.setCursor(2, 1);
  // lcd.print("Testing");

  Serial.begin(9600);

  // Create the ESP32's own WiFi network (Access Point mode).
  // Unlike WiFi.begin(), this doesn't need to wait/retry - it's
  // essentially instant since the ESP32 IS the network now.
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("Access Point started. Network name: ");
  Serial.println(AP_SSID);
  Serial.print("ESP32 IP address (for reference): ");
  Serial.println(WiFi.softAPIP()); // usually 192.168.4.1

  udp.begin(GAME_PORT); // not required for sending-only, but harmless to open
}

void loop(void) {
  float distance = readDistanceCM();

  // lcd.clear();
  // lcd.setCursor(0,0);
  // lcd.print("Distance");
  // lcd.setCursor(0,1);
  // lcd.print(distance);
  // lcd.print("cm");

  Serial.println(distance);

  switch (currentState) {
    case IDLE:
      if (distance > 75 && distance < 160) {
        currentState = PLAYING;
      }
      if (distance >= 10 && distance <= 75) {
        currentState = PLAYING;
      }
      break;

    case PLAYING:
      if (distance > 160 || distance < 10) {
        Serial.println("Out of Bounds");
        currentState = IDLE;
      } else {
        // For now, sending distance as a placeholder "x" with y=0,
        // since we're not calculating real x/y yet. See the bottom
        // of this file for what to add when that's ready.
        sendPosition(distance, 0.0);
      }
      break;
  }

  // for (int pos = 0; pos <= 180; pos += 1) {
  //   motor1.write(pos);
  //   delay(15);
  // }
  // for (int pos = 180; pos >= 0; pos -= 1) {
  //   motor1.write(pos);
  //   delay(15);
  // }

  delay(500);
}

// Reads distance from the Sensor in centimeters.
// Returns -1 if the echo timed out (nothing detected in range).
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2.0;
}

// Sends the computed x/y position to the Java game over WiFi
// using UDP.
void sendPosition(float x, float y) {
  char payload[100];
  snprintf(payload, sizeof(payload), "{\"x\":%.2f,\"y\":%.2f}", x, y);

  udp.beginPacket(GAME_IP, GAME_PORT);
  udp.print(payload);
  udp.endPacket();
}

/*

  case PLAYING:
    if (distance > 160 || distance < 10) {
      Serial.println("Out of Bounds");
      currentState = IDLE;
    } else {
      // Example for a single angled sensor (see SENSOR_ANGLE_DEG
      // discussion from earlier) - replace 0.0 with this sensor's
      // known fixed mounting angle in degrees:
      float angleRad = radians(0.0);
      float x = distance * sin(angleRad);   // + SENSOR_X offset if not at origin
      float y = distance * cos(angleRad);

      sendPosition(x, y);
    }
    break;


  ---------------------------------------------------------------
*/
s