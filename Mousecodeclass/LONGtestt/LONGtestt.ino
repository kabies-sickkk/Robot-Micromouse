#include <Arduino.h>

// Cấu hình chân
const uint8_t ENCODER_LEFT_CLK = 2;
const uint8_t ENCODER_RIGHT_CLK = 3;
const uint8_t LED = 6;
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
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;

// PID tham số
float Kp = 0.5;  // Hệ số tỉ lệ
float Ki = 0.0;  // Hệ số tích phân
float Kd = 0.05; // Hệ số vi phân
const float Ts = 0.05; // Chu kỳ lấy mẫu (50ms)

// Setpoint
const int desiredSensorLeft = 75; // Khoảng cách mong muốn từ tường bên trái

// PID biến
float sumErrorLeft = 0;
float previousErrorLeft = 0;

// Biến điều khiển
float baseFwd = 120;  // Tốc độ cơ bản
float turn = 0;
bool missLeftWall = false;
bool running = false;
int cnt = 0;

// Hàm đọc cảm biến
int readSensor(uint8_t pin) {
  int value = analogRead(pin);
  return constrain(value, 20, 500); // Hạn chế giá trị trong khoảng hợp lý
}

// Hàm điều chỉnh PID
float PID_LeftWall(int sensorLeft, float desiredSensorLeft, float Kp, float Ki, float Kd, float Ts) {
  float errorLeft = desiredSensorLeft - (float)sensorLeft;
  sumErrorLeft += errorLeft;
  float turn_ = Kp * errorLeft + Ki * sumErrorLeft * Ts + Kd * (errorLeft - previousErrorLeft) / Ts;
  previousErrorLeft = errorLeft;
  turn_ = constrain(turn_, -50, 50); // Hạn chế độ quay
  return turn_;
}

// Hàm điều khiển động cơ
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
  } else {
    digitalWrite(MOTOR_LEFT_DIR, LOW);
    leftSpeed = -leftSpeed;
  }

  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_RIGHT_DIR, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);
    rightSpeed = -rightSpeed;
  }

  analogWrite(MOTOR_LEFT_PWM, constrain(leftSpeed, 0, 255));
  analogWrite(MOTOR_RIGHT_PWM, constrain(rightSpeed, 0, 255));
}

// Hàm xác định trạng thái tường
int getWallState(int sensorLeft, int sensorMid, int threshold) {
  if (sensorMid > threshold) {
    return 0; // Không có tường trước
  }
  return 1; // Có tường trước
}

// Hàm setup() để cấu hình các chân và cài đặt ban đầu
void setup() {
  // Khởi tạo các chân làm đầu vào/ra
  pinMode(ENCODER_LEFT_CLK, INPUT);
  pinMode(ENCODER_RIGHT_CLK, INPUT);
  pinMode(LED, OUTPUT);
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
  pinMode(FUNCTION_PIN, INPUT);
  pinMode(BATTERY_VOLTS, INPUT);

  // Khởi tạo Serial Monitor để kiểm tra thông tin debug
  Serial.begin(9600);
}

// Vòng lặp chính
void loop() {
  int sensorLeft = readSensor(SENSOR_1); // Cảm biến trái
  int sensorMid = readSensor(SENSOR_0); // Cảm biến trước bên trái
  int state = getWallState(sensorLeft, sensorMid, 100);

  if (running) {
    if (!missLeftWall) {
      if (state == 0) { // Không có tường trước
        turn = PID_LeftWall(sensorLeft, desiredSensorLeft, Kp, Ki, Kd, Ts);
        baseFwd = 120;
      } else { // Gặp tường trước
        sumErrorLeft = 0;
        baseFwd = 0;
        turn = 110; // Quay lại
      }
    } else {
      sumErrorLeft = 0;
      cnt++;
      if (cnt <= 5) {
        baseFwd = 80;
        turn = 0;
      } else if (cnt <= 7) {
        baseFwd = 0;
        turn = -100; // Quay lại
      } else {
        baseFwd = 100;
        turn = -65; // Quay đi tiếp
      }
      if (sensorLeft <= 60) {
        cnt = 0;
        missLeftWall = false;
        baseFwd = 0;
        turn = 0;
      }
    }
  }

  // Điều khiển động cơ
  setMotorSpeed(baseFwd + turn, baseFwd - turn);

  // Debug thông tin
  Serial.print("Error Left:");
  Serial.print(previousErrorLeft);
  Serial.print(", Left:");
  Serial.print(sensorLeft);
  Serial.print("mm, Mid:");
  Serial.print(sensorMid);
  Serial.print("mm, LeftMotor:");
  Serial.print(baseFwd + turn);
  Serial.print(", RightMotor: ");
  Serial.println(baseFwd - turn);

  delay(50); // Chu kỳ lấy mẫu 50ms
}
