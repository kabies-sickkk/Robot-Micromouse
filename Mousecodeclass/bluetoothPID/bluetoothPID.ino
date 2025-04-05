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
const unsigned long TURN_TIME_90_DEGREES = 248;       // ms
const unsigned long TURN_TIME_180_DEGREES = 235 * 2;  // ms
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

// Các hàm điều khiển động cơ
void turnLeft90Degrees() {
  setMotorSpeed(-255, 255);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
  Serial.println("Hoàn thành quay trái 90 độ");
}

void turnRight90Degrees() {
  setMotorSpeed(255, -255);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
  Serial.println("Hoàn thành quay phải 90 độ");
}

void turn180Degrees() {
  setMotorSpeed(255, -255);
  delay(TURN_TIME_180_DEGREES);
  stopMotors();
  Serial.println("Hoàn thành quay 180 độ");
}

void moveForwardPID() {
  int baseSpeed = 200;
  float sensorValue = analogRead(A0);
  int pidOutput = calculatePID(sensorValue);
  setMotorSpeed(baseSpeed + pidOutput, baseSpeed - pidOutput);
  delay(TURN_TIME_CELL);
  stopMotors();
  Serial.println("Đi thẳng PID");
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

  digitalWrite(EMITTER_A, HIGH);
  digitalWrite(EMITTER_B, HIGH);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    
    // Lệnh hỏi: Hiển thị các lệnh và tác dụng của chúng
    if (command == '?') {
      Serial.println("Các lệnh điều khiển:");
      Serial.println("'S' - Đọc giá trị cảm biến và hiển thị");
      Serial.println("'F' - Tiến về phía trước");
      Serial.println("'L' - Quay trái 90 độ");
      Serial.println("'R' - Quay phải 90 độ");
      Serial.println("'B' - Quay 180 độ");
      Serial.println("'X' - Dừng động cơ");
      Serial.println("Gửi lệnh tương ứng để điều khiển robot.");
    }
    // Đọc giá trị cảm biến khi có lệnh 'S'
    else if (command == 'S') {
      int sensorFrontLeft = analogRead(SENSOR_0);
      int sensorLeft = analogRead(SENSOR_1);
      int sensorRight = analogRead(SENSOR_2);
      int sensorFrontRight = analogRead(SENSOR_3);
      
      Serial.print("Front Left: ");
      Serial.print(sensorFrontLeft);
      Serial.print(" | Left: ");
      Serial.print(sensorLeft);
      Serial.print(" | Right: ");
      Serial.print(sensorRight);
      Serial.print(" | Front Right: ");
      Serial.println(sensorFrontRight);
    }
    else {
      // Xử lý lệnh điều khiển động cơ
      switch (command) {
        case 'F':
          moveForwardPID();
          break;
        case 'L':
          turnLeft90Degrees();
          break;
        case 'R':
          turnRight90Degrees();
          break;
        case 'B':
          turn180Degrees();
          break;
        case 'X':
          stopMotors();
          Serial.println("Dừng động cơ");
          break;
      }
    }
  }
}
