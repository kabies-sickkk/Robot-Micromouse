#include <Arduino.h>

// Cấu hình chân
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;

// Thời gian quay 90 độ (điều chỉnh theo thực tế)
const unsigned long TURN_TIME_180_DEGREES = 280*2;  // Đơn vị: ms (thử nghiệm để xác định giá trị chính xác)

void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);

  Serial.begin(9600);  // Debug qua Serial
}

void loop() {
  // Quay trái 90 độ, chờ 3 giây rồi quay tiếp
  turnLeft90Degrees();
  delay(3000);  // Dừng 3 giây trước khi thực hiện lại
}

void turnLeft90Degrees() {
  // Bánh trái lùi
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);

  // Bánh phải tiến
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, 200);

  // Quay trong thời gian cố định
  delay(TURN_TIME_180_DEGREES);

  // Dừng động cơ
  stopMotors();

  Serial.println("Hoàn thành quay trái 180 độ");
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}
