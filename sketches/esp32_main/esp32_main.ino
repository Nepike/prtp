#include <WiFi.h>
#include <AsyncUDP.h>
#include <RPLidar.h>

// ========== CONFIG SECTION ========== //
const char* ssid = "Nopike";
const char* password = "11111111";
const IPAddress laptopIP(192, 168, 1, 100);  // Укажите IP вашего ноутбука
const int udpOdomPort = 1234;                // Порт для одометрии
const int udpLidarPort = 1235;               // Порт для лидара
// ==================================== //

// Hardware Pins
#define MOTOR_LEFT_A 25
#define MOTOR_LEFT_B 26
#define MOTOR_RIGHT_A 13
#define MOTOR_RIGHT_B 14
#define ENCODER_LEFT_A 35
#define ENCODER_LEFT_B 34
#define ENCODER_RIGHT_A 32
#define ENCODER_RIGHT_B 33

// Globals
AsyncUDP udp;
RPLidar lidar;
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;
float xPos = 0.0, yPos = 0.0, theta = 0.0;

// ========== ENCODER INTERRUPTS ========== //
void IRAM_ATTR leftEncoderISR() {
  digitalRead(ENCODER_LEFT_B) ? leftEncoder++ : leftEncoder--;
}

void IRAM_ATTR rightEncoderISR() {
  digitalRead(ENCODER_RIGHT_B) ? rightEncoder++ : rightEncoder--;
}

// ========== WIFI CONNECTION ========== //
void connectWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ========== LIDAR HANDLING ========== //
void processLidar() {
  if (IS_OK(lidar.waitPoint())) {
    String lidarData;
    lidarData.reserve(1024);

    size_t count = lidar.getScanData().size();
    lidarData = String(count) + ";";

    for (size_t pos = 0; pos < count; pos++) {
      float angle = lidar.getScanData()[pos].angle;
      float distance = lidar.getScanData()[pos].distance;
      lidarData += String(angle, 2) + "," + String(distance, 2) + ";";
    }

    udp.broadcastTo(lidarData.c_str(), udpLidarPort);
  } else {
    lidar.startScan();
    Serial.println("Lidar scan restarting...");
  }
}

// ========== ODOMETRY CALCULATION ========== //
void updateOdometry() {
  static long lastLeft = 0, lastRight = 0;
  const float WHEEL_DIAMETER = 0.069;    // meters
  const float WHEEL_BASE = 0.25;         // meters
  const float TICKS_PER_REV = 330.0;
  const float MM_PER_TICK = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

  long deltaLeft = leftEncoder - lastLeft;
  long deltaRight = rightEncoder - lastRight;
  lastLeft = leftEncoder;
  lastRight = rightEncoder;

  float distLeft = deltaLeft * MM_PER_TICK / 1000.0;  // meters
  float distRight = deltaRight * MM_PER_TICK / 1000.0; // meters

  theta += (distRight - distLeft) / WHEEL_BASE;
  xPos += (distLeft + distRight) / 2.0 * cos(theta);
  yPos += (distLeft + distRight) / 2.0 * sin(theta);
}

// ========== MAIN SETUP ========== //
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, 21, 22); // Лидар на пинах 21(RX), 22(TX)
  lidar.begin(Serial1);

  // Настройка пинов
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);

  pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
  pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
  pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);

  // Прерывания энкодеров
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), rightEncoderISR, RISING);

  connectWiFi();

  // Настройка UDP
  if (!udp.listen(udpOdomPort)) {
    Serial.println("UDP listen failed!");
    while(1) delay(1000);
  }
}

// ========== MAIN LOOP ========== //
void loop() {
  static uint32_t lastOdomTime = 0;
  static uint32_t lastLidarTime = 0;

  // Отправка одометрии каждые 100ms
  if (millis() - lastOdomTime >= 100) {
    String odomJSON;
    odomJSON.reserve(256);
    odomJSON = "{\"x\":" + String(xPos, 3) +
               ",\"y\":" + String(yPos, 3) +
               ",\"theta\":" + String(theta, 3) + "}";

    udp.broadcastTo(odomJSON.c_str(), udpOdomPort);
    lastOdomTime = millis();
  }

  // Отправка данных лидара каждые 50ms
  if (millis() - lastLidarTime >= 50) {
    processLidar();
    lastLidarTime = millis();
  }

  // Проверка подключения WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    connectWiFi();
  }

  // Обновление одометрии
  updateOdometry();

  delay(1);
}
