#include <WiFi.h>
#include <AsyncUDP.h>

// Конфигурация сети
const char* ssid = "Nopike";
const char* password = "11111111";
const int udpPort = 1234;
AsyncUDP udp;

// Определение пинов
#define LEFT_MOTOR_A 25     
#define LEFT_MOTOR_B 26     
#define RIGHT_MOTOR_A 13    
#define RIGHT_MOTOR_B 14    
#define LEFT_ENCODER_A 35    
#define LEFT_ENCODER_B 34    
#define RIGHT_ENCODER_A 32   
#define RIGHT_ENCODER_B 33    

// Энкодеры и управление
volatile long left_encoder_value = 0;
volatile long right_encoder_value = 0;
long last_left_encoder = 0;
long last_right_encoder = 0;
long last_speed_left_encoder = 0, last_speed_right_encoder = 0;

// Фильтрация скорости
float vlSpeedFiltered = 0.0, vrSpeedFiltered = 0.0;
uint32_t lastTimeL = 0, lastTimeR = 0;

// Целевые скорости
float targetLeftWheelSpeed = 0, targetRightWheelSpeed = 0;

// Одометрия
float xPos = 0.0, yPos = 0.0, theta = 0.0;

// Константы
const float WHEEL_DIAMETER = 69.0;
const float WHEEL_BASE = 250.0;
const float ENCODER_RESOLUTION = 330.0;
const float TICKS_TO_MM = (PI * WHEEL_DIAMETER) / ENCODER_RESOLUTION;
const float ALPHA = 0.5;

// PID параметры
float pidKp = 1.1;
float pidKi = 1.3;
float pidKd = 0.01;
float pidKff = 0.25;
float errorLeftIntegral = 0;
float errorRightIntegral = 0;
float prevErrorLeft = 0;
float prevErrorRight = 0;
uint32_t lastPIDTime = 0;

// Компенсация движения
bool goingStraight = false;
long leftBase = 0;
long rightBase = 0;

// Прерывания энкодеров
void left_interrupt() { digitalRead(LEFT_ENCODER_B) ? left_encoder_value++ : left_encoder_value--; }
void right_interrupt() { digitalRead(RIGHT_ENCODER_B) ? right_encoder_value++ : right_encoder_value--; }

void updateOdometry() {
  long deltaLeft = left_encoder_value - last_left_encoder;
  long deltaRight = right_encoder_value - last_right_encoder;
  last_left_encoder = left_encoder_value;
  last_right_encoder = right_encoder_value;

  float distLeft = deltaLeft * TICKS_TO_MM;
  float distRight = deltaRight * TICKS_TO_MM;
  float deltaS = (distLeft + distRight)/2.0;
  float deltaTheta = (distRight - distLeft)/WHEEL_BASE;

  theta += deltaTheta;
  xPos += deltaS * cos(theta);
  yPos += deltaS * sin(theta);
}

void computeSpeed() {
  uint32_t now = micros();
  if(lastTimeL == 0 || lastTimeR == 0) { 
    lastTimeL = lastTimeR = now;
    last_speed_left_encoder = left_encoder_value;
    last_speed_right_encoder = right_encoder_value;
    return;
  }

  float dtL = (now - lastTimeL) * 1e-6f;
  float dtR = (now - lastTimeR) * 1e-6f;

  if(dtL > 0 && left_encoder_value != last_speed_left_encoder) {
    long deltaLeft = left_encoder_value - last_speed_left_encoder;
    float vlSpeed = ((float)deltaLeft/ENCODER_RESOLUTION)*(PI*WHEEL_DIAMETER)/dtL;
    vlSpeedFiltered = ALPHA*vlSpeed + (1.0f-ALPHA)*vlSpeedFiltered;
    last_speed_left_encoder = left_encoder_value;
    lastTimeL = now;
  } else if(dtL > 0.005) vlSpeedFiltered = 0;

  if(dtR > 0 && right_encoder_value != last_speed_right_encoder) {
    long deltaRight = right_encoder_value - last_speed_right_encoder;
    float vrSpeed = ((float)deltaRight/ENCODER_RESOLUTION)*(PI*WHEEL_DIAMETER)/dtR;
    vrSpeedFiltered = ALPHA*vrSpeed + (1.0f-ALPHA)*vrSpeedFiltered;
    last_speed_right_encoder = right_encoder_value;
    lastTimeR = now;
  } else if(dtR > 0.005) vrSpeedFiltered = 0;
}

void resetPID() {
  errorLeftIntegral = errorRightIntegral = 0;
  prevErrorLeft = prevErrorRight = 0;
  vlSpeedFiltered = vrSpeedFiltered = 0;
  lastPIDTime = millis();
}

void computeWheelsPID() {
  const float integralLimit = 30.0;
  static float lastTargetLeft = 0.0, lastTargetRight = 0.0;

  if(targetLeftWheelSpeed != lastTargetLeft || 
     targetRightWheelSpeed != lastTargetRight ||
     (targetLeftWheelSpeed == 0 && targetRightWheelSpeed == 0)) resetPID();

  lastTargetLeft = targetLeftWheelSpeed;
  lastTargetRight = targetRightWheelSpeed;

  uint32_t now = millis();
  float dt = (now - lastPIDTime)/1000.0f;
  if(dt < 0.001f) dt = 0.001f;
  lastPIDTime = now;

  float errorLeft = targetLeftWheelSpeed - vlSpeedFiltered;
  float errorRight = targetRightWheelSpeed - vrSpeedFiltered;

  errorLeftIntegral = constrain(errorLeftIntegral + errorLeft*dt, -integralLimit, integralLimit);
  errorRightIntegral = constrain(errorRightIntegral + errorRight*dt, -integralLimit, integralLimit);

  float derivativeLeft = (errorLeft - prevErrorLeft)/dt;
  float derivativeRight = (errorRight - prevErrorRight)/dt;
  prevErrorLeft = errorLeft;
  prevErrorRight = errorRight;

  float outputLeft = pidKp*errorLeft + pidKi*errorLeftIntegral + pidKd*derivativeLeft + pidKff*targetLeftWheelSpeed;
  float outputRight = pidKp*errorRight + pidKi*errorRightIntegral + pidKd*derivativeRight + pidKff*targetRightWheelSpeed;

  if(goingStraight && fabs(targetLeftWheelSpeed) > 1.0 && fabs(targetRightWheelSpeed) > 1.0) {
    long diff = (left_encoder_value - leftBase) - (right_encoder_value - rightBase);
    float corr = 3.0f * diff;
    outputLeft -= corr*0.5f;
    outputRight += corr*0.5f;
  }

  int leftA = constrain(outputLeft >=0 ? 0 : (int)-outputLeft, 0, 255);
  int leftB = constrain(outputLeft >=0 ? (int)outputLeft : 0, 0, 255);
  int rightA = constrain(outputRight >=0 ? 0 : (int)-outputRight, 0, 255);
  int rightB = constrain(outputRight >=0 ? (int)outputRight : 0, 0, 255);

  analogWrite(LEFT_MOTOR_A, leftA);
  analogWrite(LEFT_MOTOR_B, leftB);
  analogWrite(RIGHT_MOTOR_A, rightA);
  analogWrite(RIGHT_MOTOR_B, rightB);
}

void setRobotVelocity(float linearVelocity, float angularVelocity) {
  if(fabs(linearVelocity) < 10 && fabs(angularVelocity) < 0.1) {
    targetLeftWheelSpeed = targetRightWheelSpeed = 0;
    resetPID();
    goingStraight = false;
    return;
  }

  goingStraight = (fabs(angularVelocity) < 0.0001f);
  if(goingStraight) {
    leftBase = left_encoder_value;
    rightBase = right_encoder_value;
  }

  targetLeftWheelSpeed = linearVelocity - angularVelocity*WHEEL_BASE/2.0;
  targetRightWheelSpeed = linearVelocity + angularVelocity*WHEEL_BASE/2.0;
}

void processCommand(String command) {
  command.trim();
  
  if(command.startsWith("SET_PWM")) {
    int vals[4];
    if(sscanf(command.c_str(), "SET_PWM %d %d %d %d", &vals[0], &vals[1], &vals[2], &vals[3]) == 4) {
      analogWrite(LEFT_MOTOR_A, constrain(vals[0], 0, 255));
      analogWrite(LEFT_MOTOR_B, constrain(vals[1], 0, 255));
      analogWrite(RIGHT_MOTOR_A, constrain(vals[2], 0, 255));
      analogWrite(RIGHT_MOTOR_B, constrain(vals[3], 0, 255));
    }
  }
  else if(command.startsWith("SET_ROBOT_VELOCITY")) {
    float lin, ang;
    if(sscanf(command.c_str(), "SET_ROBOT_VELOCITY %f %f", &lin, &ang) == 2) {
      setRobotVelocity(lin, ang);
    }
  }
  else if(command.startsWith("SET_COEFF")) {
    float kp, ki, kd, kff;
    if(sscanf(command.c_str(), "SET_COEFF %f %f %f %f", &kp, &ki, &kd, &kff) == 4) {
      pidKp = kp; pidKi = ki; pidKd = kd; pidKff = kff;
    }
  }
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) delay(500);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), left_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), right_interrupt, RISING);

  connectToWiFi();
  udp.listen(udpPort);
  udp.onPacket([](AsyncUDPPacket packet) {
    processCommand(String((char*)packet.data()));
    packet.printf("OK");
  });
}

void loop() {
  static uint32_t tPID = 0, tStatus = 0;
  uint32_t now = millis();

  if(now - tPID >= 50) {
    computeSpeed();
    updateOdometry();
    computeWheelsPID();
    tPID = now;
  }

  if(now - tStatus >= 200) {
    String status = "{\"x\":" + String(xPos, 3) + 
                    ",\"y\":" + String(yPos, 3) + 
                    ",\"theta\":" + String(theta, 3) + 
                    "}";
    udp.broadcast(status.c_str());
    tStatus = now;
  }
}