#include <Wire.h>
#include <VL53L0X.h>
const uint8_t sensorCount = 3;
const uint8_t xshutPins[sensorCount] = { 4, 2, 15 };
VL53L0X sensors[sensorCount];
const int ledPin1 = 25;
const int ledPin2 = 13;
const int freq = 5000;
const int resolution = 8;
#define DIR1_1 27
#define DIR1_2 26
#define DIR2_1 12
#define DIR2_2 14
float sumErrorLeft = 0;
float previousErrorLeft = 0;
float previousSensorLeft = 0;
int state = 0;
int baseFwd = 0;
int turn = 0;
bool missLeftWall = false;
bool running = false;
int cnt = 0;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  for (uint8_t i = 0; i < sensorCount; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCount; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(100);
    sensors[i].setTimeout(100);
    if (!sensors[i].init()) {
      Serial.print("Không phát hiện và khởi tạo được cảm biến ");
      Serial.println(i);
      while (1)
        ;
    }
    sensors[i].setAddress(0x50 + i);
    sensors[i].startContinuous();
  }
  ledcAttach(ledPin1, freq, resolution);
  ledcAttach(ledPin2, freq, resolution);
  pinMode(DIR1_1, OUTPUT);
  pinMode(DIR1_2, OUTPUT);
  pinMode(DIR2_1, OUTPUT);
  pinMode(DIR2_2, OUTPUT);
  ledcWrite(ledPin1, 0);
  ledcWrite(ledPin2, 0);
  running = false;
}
void Right_Motor_PWM(float PWM_) {
  if (PWM_ >= 0) {
    ledcWrite(ledPin1, PWM_);
    digitalWrite(DIR1_1, 1);
    digitalWrite(DIR1_2, 0);
  } else {
    ledcWrite(ledPin1, abs(PWM_));
    digitalWrite(DIR1_1, 0);
    digitalWrite(DIR1_2, 1);
  }
}
void Left_Motor_PWM(float PWM_) {
  if (PWM_ >= 0) {
    ledcWrite(ledPin2, PWM_);
    digitalWrite(DIR2_1, 1);
    digitalWrite(DIR2_2, 0);
  } else {
    ledcWrite(ledPin2, abs(PWM_));
    digitalWrite(DIR2_1, 0);
    digitalWrite(DIR2_2, 1);
  }
}
int getWallState(int sensorLeft, int sensorMid, int sensorRight, int threshold) {
  int wallState;
  if (sensorMid > threshold)
    wallState = 0;
  else
    wallState = 1;
  return wallState;
}
float PID_LeftWall(int sensorLeft, float desiredSensorLeft, float Kp, float Ki, float Kd, float Ts) {
  float errorLeft = desiredSensorLeft - (float)sensorLeft;
  sumErrorLeft += errorLeft;
  float turn_ = Kp * errorLeft + Ki * sumErrorLeft * Ts + Kd * (errorLeft - previousErrorLeft) / Ts;
  previousErrorLeft = errorLeft;
  turn_ = constrain(turn_, -50, 50);
  return turn_;
}
void loop() {
  int sensorLeft = sensors[0].readRangeContinuousMillimeters() - 40;
  int sensorMid = sensors[1].readRangeContinuousMillimeters() - 20;
  int sensorRight = sensors[2].readRangeContinuousMillimeters() - 20;
  sensorLeft = constrain(sensorLeft, 20, 500);
  sensorMid = constrain(sensorMid, 20, 500);
  sensorRight = constrain(sensorRight, 20, 500);
  if (sensorRight < 30) running = true;
  int state = getWallState(sensorLeft, sensorMid, sensorRight, 100);
  if (sensorLeft > 150) missLeftWall = true;
  if (running) {
    if (!missLeftWall) {
      if (state == 0) {
        if (sensorMid > 490) {
          turn = PID_LeftWall(sensorLeft, 75.0, 0.5, 0.0, 0.05, 0.05);
          baseFwd = 199;
        } else {
          turn = PID_LeftWall(sensorLeft, 75.0, 0.5, 0.0, 0.05, 0.05);
          baseFwd = 120;
        }
      } else {
        sumErrorLeft = 0;
        baseFwd = 0;
        turn = 110;
      }
    } else {
      sumErrorLeft = 0;
      cnt++;
      if (cnt <= 5) {
        baseFwd = 80;
        turn = 0;
      } else if (cnt <= 7) {
        baseFwd = 0;
        turn = -100;
      } else {
        baseFwd = 100;
        turn = -65;
      }
      if (sensorLeft <= 60) {
        cnt = 0;
        missLeftWall = false;
        baseFwd = 0;
        turn = 0;
      }
    }
  }
  Left_Motor_PWM(baseFwd + turn);
  Right_Motor_PWM(baseFwd - turn);
  Serial.print("Error Left:");
  Serial.print(previousErrorLeft);
  Serial.print(", Left:");
  Serial.print(sensorLeft);
  Serial.print("mm, Mid:");
  Serial.print(sensorMid);
  Serial.print("mm, Right:");
  Serial.print(sensorRight);
  Serial.print("mm, LeftMotor:");
  Serial.print(baseFwd + turn);
  Serial.print(", RightMotor: ");
  Serial.println(baseFwd - turn);
  delay(50);
}