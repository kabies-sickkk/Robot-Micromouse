#include <Arduino.h>

// Cấu hình chân
const uint8_t ENCODER_LEFT_CLK = 2;
const uint8_t ENCODER_RIGHT_CLK = 3;
const uint8_t ENCODER_LEFT_B = 4;
const uint8_t ENCODER_RIGHT_B = 5;
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;
const uint8_t EMITTER_A = 11;
const uint8_t EMITTER_B = 12;

const uint8_t SENSOR_0 = A3; // Cảm biến trước bên trái
const uint8_t SENSOR_1 = A2 ; // Cảm biến trái
const uint8_t SENSOR_2 = A1; // Cảm biến phải
const uint8_t SENSOR_3 = A0; // Cảm biến trước bên phải

const uint8_t SWITCHES_PIN = A6;
const uint8_t BATTERY_PIN = A7;

// Ngưỡng cảm biến hiệu chuẩn
const int SENSOR_FRONT_LEFT_THRESHOLD = 112;  // SENSOR_0
const int SENSOR_FRONT_RIGHT_THRESHOLD = 78; // SENSOR_3
const int SENSOR_LEFT_THRESHOLD = 359;        // SENSOR_1
const int SENSOR_RIGHT_THRESHOLD = 150;       // SENSOR_2

// PID
float Kp = 0.35, Ki = 0.0, Kd = 0.03; // Hằng số PID
float setPoint = 80;               // Khoảng cách lý tưởng đến tường
float error = 0, previousError = 0, integral = 0, derivative = 0;

const unsigned long TURN_TIME_90_DEGREES = 276 ; // Đơn vị: ms (thử nghiệm để xác định giá trị chính xác)
// Tốc độ cơ bản
int baseSpeed = 150;


void setup() {
  // Cấu hình các chân
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

  digitalWrite(EMITTER_A, HIGH); // Bật nguồn cho cảm biến
  digitalWrite(EMITTER_B, HIGH);

  Serial.begin(9600); // Khởi tạo giao tiếp Serial
}

void loop() {
  // Đọc giá trị cảm biến
  int sensorFrontLeft = analogRead(SENSOR_0);
  int sensorLeft = analogRead(SENSOR_1);
  int sensorRight = analogRead(SENSOR_2);
  int sensorFrontRight = analogRead(SENSOR_3);

  // Hiển thị dữ liệu cảm biến
  Serial.print("Front Left: "); Serial.print(sensorFrontLeft);
  Serial.print(" | Left: "); Serial.print(sensorLeft);
  Serial.print(" | Right: "); Serial.print(sensorRight);
  Serial.print(" | Front Right: "); Serial.println(sensorFrontRight);

  // Phát hiện ngõ cụt (chướng ngại vật phía trước)
  if (sensorFrontLeft > SENSOR_FRONT_LEFT_THRESHOLD && sensorFrontRight > SENSOR_FRONT_RIGHT_THRESHOLD) {
    Serial.println("Obstacle detected! Turning around...");
    turnAround(); // Quay đầu
    return;       // Thoát khỏi vòng lặp hiện tại để tránh điều khiển động cơ tiếp
  }

  // Tính toán PID cho bám tường bên trái
  error = setPoint - sensorLeft;
  integral += error;
  derivative = error - previousError;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  previousError = error;

  // Điều chỉnh tốc độ động cơ
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Giới hạn tốc độ
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Điều khiển động cơ
  motorControl(leftSpeed, rightSpeed);
  delay(50);
}

// Hàm điều khiển động cơ
void motorControl(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(MOTOR_LEFT_DIR,HIGH);
    analogWrite(MOTOR_LEFT_PWM, leftSpeed);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
    analogWrite(MOTOR_LEFT_PWM, -leftSpeed);
  }

  if (rightSpeed > 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
    analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
    analogWrite(MOTOR_RIGHT_PWM, -rightSpeed);
  }
}

// Hàm quay đầu
void turnAround() {
  // Dừng động cơ
  motorControl(0, 0);
  delay(500);

  // Quay robot 180 độ
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 250);
  analogWrite(MOTOR_RIGHT_PWM, 250);

  delay(264); // Điều chỉnh thời gian để quay đủ 180 độ

  // Dừng động cơ
  motorControl(0, 0);
  delay(500);
}
