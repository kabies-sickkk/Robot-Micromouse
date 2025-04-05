#include <Arduino.h>

// Cấu hình chân động cơ
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;

const uint8_t EMITTER_A = 11;
const uint8_t EMITTER_B = 12;

const uint8_t SENSOR_0 = A3;  // Cảm biến trước bên trái
const uint8_t SENSOR_1 = A2;  // Cảm biến trái
const uint8_t SENSOR_2 = A1;  // Cảm biến phải
const uint8_t SENSOR_3 = A0;  // Cảm biến trước bên phải

// Thời gian di chuyển và quay
const unsigned long TURN_TIME_90_DEGREES = 240;       // ms
const unsigned long TURN_TIME_180_DEGREES = 225 * 2;  // ms
const unsigned long TURN_TIME_CELL = 655;             // ms

// PID điều khiển
const float KP = 0.125, KI = 0.0, KD = 0.05;
const float SET_POINT = 80;  // Giả định giá trị mong muốn từ cảm biến
float error = 0, previousError = 0, integral = 0, derivative = 0;

// Hàm thiết lập tốc độ động cơ
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  digitalWrite(MOTOR_LEFT_DIR, leftSpeed > 0 ? HIGH : LOW);
  analogWrite(MOTOR_LEFT_PWM, constrain(abs(leftSpeed), 0, 255));

  digitalWrite(MOTOR_RIGHT_DIR, rightSpeed > 0 ? LOW : HIGH);
  analogWrite(MOTOR_RIGHT_PWM, constrain(abs(rightSpeed), 0, 255));
}

// Hàm dừng động cơ
void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

// Hàm tính toán PID
int calculatePID(float sensorValue) {
  error = SET_POINT - sensorValue;
  integral += error;
  derivative = error - previousError;
  previousError = error;

  return KP * error + KI * integral + KD * derivative;
}

// Hàm di chuyển thẳng với PID
void turnGoPID() {
  int baseSpeed = 200;
  float sensorValue = analogRead(A0);  // Đọc giá trị từ cảm biến
  int pidOutput = calculatePID(sensorValue);

  int leftSpeed = baseSpeed + pidOutput;
  int rightSpeed = baseSpeed - pidOutput;

  setMotorSpeed(leftSpeed, rightSpeed);
  delay(TURN_TIME_CELL);
  stopMotors();
  Serial.println("Hoàn thành di thẳng với PID");
}

// Hàm quay trái 90 độ
void turnLeft90Degrees() {
  setMotorSpeed(-255, 255);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
  Serial.println("Hoàn thành quay trái 90 độ");
}

// Hàm quay phải 90 độ
void turnRight90Degrees() {
  setMotorSpeed(255, -255);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
  Serial.println("Hoàn thành quay phải 90 độ");
}

// Hàm quay 180 độ
void turn180Degrees() {
  setMotorSpeed(255, -255);
  delay(TURN_TIME_180_DEGREES);
  stopMotors();
  Serial.println("Hoàn thành quay 180 độ");
}

void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);

  pinMode(SENSOR_0, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);

  digitalWrite(EMITTER_A, HIGH);  // Bật nguồn cho cảm biến
  digitalWrite(EMITTER_B, HIGH);

  Serial.begin(9600);
}

void loop() {
  // // Đọc giá trị cảm biến
  // int sensorFrontLeft = analogRead(SENSOR_0);
  // int sensorLeft = analogRead(SENSOR_1);
  // int sensorRight = analogRead(SENSOR_2);
  // int sensorFrontRight = analogRead(SENSOR_3);

  // // Hiển thị dữ liệu cảm biến
  // Serial.print("Front Left: ");
  // Serial.print(sensorFrontLeft);
  // Serial.print(" | Left: ");
  // Serial.print(sensorLeft);
  // Serial.print(" | Right: ");
  // Serial.print(sensorRight);
  // Serial.print(" | Front Right: ");
  // Serial.println(sensorFrontRight);

  // turnLeft90Degrees();
  // delay(5000);

  turnGoPID();
  delay(5000);

  // turnRight90Degrees();
  // delay(5000);

  // turn180Degrees();
  // delay(5000);
}
