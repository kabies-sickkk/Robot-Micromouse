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


// Ngưỡng cảm biến
const int SENSOR_FRONT_LEFT_THRESHOLD = 112;
const int SENSOR_FRONT_RIGHT_THRESHOLD = 78;
const int SENSOR_LEFT_THRESHOLD = 359;
const int SENSOR_RIGHT_THRESHOLD = 150;

// Thời gian quay 90 độ (ms)
const unsigned long TURN_TIME_90_DEGREES = 230;  // Đơn vị: ms (thử nghiệm để xác định giá trị chính xác) quayvoi toc do 255
const unsigned long TURN_TIME_180_DEGREES = 220*2;  // Đơn vị: ms (thử nghiệm để xác định giá trị chính xác) quayvoi toc do 255
const unsigned long TURN_TIME_CELL = 700; // chay tien toi giua o

// Tốc độ cơ bản
const int BASE_SPEED = 200;

// PID điều khiển
const float KP = 0.11, KI = 0.0, KD = 0.05;
const float SET_POINT = 80;

float error = 0, previousError = 0, integral = 0, derivative = 0;



bool isFunctionActivated = false; // Trạng thái kích hoạt chương trình
// Hàm kiểm tra nút Function
void checkFunctionButton() {
    int functionValue = analogRead(FUNCTION_PIN);

    if (functionValue == 1023) { 
        digitalWrite(LED, LOW);
        delay(2000);
        isFunctionActivated = true; // Đánh dấu chương trình đã được kích hoạt
        Serial.println("Nút Function đã được kích hoạt. Chương trình bắt đầu.");
    } else {
        stopMotors();
        Serial.println("Chờ kích hoạt nút Function...");
        digitalWrite(LED, HIGH);
        delay(500); // Tránh kiểm tra quá thường xuyên
    }
}


// Hàm bật mắt phát
void enableEmitters() {
  digitalWrite(EMITTER_A, HIGH);
  digitalWrite(EMITTER_B, HIGH);
}

// Hàm tắt mắt phát
void disableEmitters() {
  digitalWrite(EMITTER_A, LOW);
  digitalWrite(EMITTER_B, LOW);
}

// Hàm đọc cảm biến
int readSensor(uint8_t pin) {
  enableEmitters();
  delayMicroseconds(100);
  int value = analogRead(pin);
  disableEmitters();
  return value;
}

// Hàm thiết lập tốc độ động cơ
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Động cơ trái
  digitalWrite(MOTOR_LEFT_DIR, leftSpeed > 0 ? HIGH : LOW);
  analogWrite(MOTOR_LEFT_PWM, constrain(abs(leftSpeed), 0, 255));

  // Động cơ phải
  digitalWrite(MOTOR_RIGHT_DIR, rightSpeed > 0 ? HIGH : LOW);
  analogWrite(MOTOR_RIGHT_PWM, constrain(abs(rightSpeed), 0, 255));
}

// Hàm dừng động cơ
void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

  // Hàm quay trái 90 độ
  void turnLeft90Degrees() {
    setMotorSpeed(-255, 255);
    delay(TURN_TIME_90_DEGREES);
    stopMotors();
    Serial.println("Hoàn thành quay trái 90 độ");
  }

    // Hàm quay trái 90 độ
  void turnGo() {
    setMotorSpeed(255, 255);
    delay(TURN_TIME_CELL);
    stopMotors();
    Serial.println("Hoàn thành di thang");
  }

  // Hàm quay phải 90 độ
  void turnRight90Degrees() {
    setMotorSpeed(255, -255);
    delay(TURN_TIME_90_DEGREES);
    stopMotors();
    Serial.println("Hoàn thành quay phải 90 độ");
  }

void turnLeft180Degrees() {
  setMotorSpeed(-255, 255);
  delay(TURN_TIME_180_DEGREES);
  stopMotors();
  Serial.println("Hoàn thành quay trái 180 độ");
}

// Tính toán điều chỉnh PID
int calculatePID(int sensorValue) {
  error = SET_POINT - sensorValue;
  integral += error;
  derivative = error - previousError;
  previousError = error;
  return KP * error + KI * integral + KD * derivative;
}

// Hàm điều khiển PID
void applyPIDControl(int sensorValue) {
  int pidOutput = calculatePID(sensorValue);

  // Tính toán tốc độ động cơ
  int leftSpeed = BASE_SPEED - pidOutput;
  int rightSpeed = BASE_SPEED + pidOutput;

  // Thiết lập tốc độ động cơ
  setMotorSpeed(leftSpeed, rightSpeed);

  // Debug
  Serial.print("Sensor Left: ");
  Serial.print(sensorValue);
  Serial.print(" | PID Output: ");
  Serial.print(pidOutput);
  Serial.print(" | Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);
}

void setup() {
  // Cấu hình chân
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(SENSOR_0, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, 0);
  // Tắt mắt phát ban đầu
  disableEmitters();

  // Serial để debug
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  if (!isFunctionActivated) {
    checkFunctionButton();  // Kiểm tra nút Function
    return;                 // Thoát sớm nếu chưa kích hoạt
  }
  // Nếu nút đã được kích hoạt, thực hiện chương trình
  // Đọc cảm biến
  int sensorFrontLeft = readSensor(SENSOR_0);
  int sensorFrontRight = readSensor(SENSOR_3);
  int sensorLeft = readSensor(SENSOR_1);
  int sensorRight = readSensor(SENSOR_2);

  // Kiểm tra va chạm phía trước
  if (sensorFrontLeft > SENSOR_FRONT_LEFT_THRESHOLD && sensorFrontRight > SENSOR_FRONT_RIGHT_THRESHOLD) {
    stopMotors();
    Serial.println("Cảm biến trước vượt ngưỡng, robot dừng lại.");
    delay(70);  // Tạm dừng để ổn định
    // Kiểm tra không gian bên để quay
    if (sensorRight < SENSOR_RIGHT_THRESHOLD) {
      Serial.println("Tường bên phải nhỏ hơn ngưỡng, quay phải.");
      turnRight90Degrees();
    } else if (sensorLeft < SENSOR_LEFT_THRESHOLD) {
      Serial.println("Tường bên trái nhỏ hơn ngưỡng, quay trái.");
      turnLeft90Degrees();
    } else {
      Serial.println("Không có không gian để quay, robot giữ nguyên vị trí.");
      while (true) {
        delay(100);  // Chờ xử lý thủ công
      }
    }
    stopMotors();
    delay(70);
  }
  // Điều khiển PID nếu không có va chạm
  applyPIDControl(sensorLeft);

  delay(10);  // Giảm tải CPU
}
