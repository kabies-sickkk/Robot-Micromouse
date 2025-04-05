// Khai báo các chân
const uint8_t ENCODER_LEFT_CLK = 2;
const uint8_t ENCODER_RIGHT_CLK = 3;
const uint8_t LED = 6;
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;
const uint8_t EMITTER_A = 11;
const uint8_t EMITTER_B = 12;
const uint8_t SENSOR_0 = A3;  // Cảm biến trước trái
const uint8_t SENSOR_1 = A2;  // Cảm biến trái
const uint8_t SENSOR_2 = A1;  // Cảm biến phải
const uint8_t SENSOR_3 = A0;  // Cảm biến trước phải
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;

// Thông số PD
float Kp = 1.2;  // Hệ số tỷ lệ
float Kd = 0.6;  // Hệ số vi phân

int setPoint = 200;  // Mục tiêu giá trị từ cảm biến trái
float previousError = 0;

// Hàm thiết lập
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

  Serial.begin(115200);  // Dùng để debug
}

// Các thông số và khai báo không thay đổi

// Hàm đọc giá trị cảm biến
float readSensor(uint8_t pin) {
  return analogRead(pin);
}

// Hàm điều khiển động cơ
void controlMotors(int baseSpeed, float correction) {
  int leftSpeed = constrain(baseSpeed - correction, 0, 255);
  int rightSpeed = constrain(baseSpeed + correction, 0, 255);

  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed));

  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed));
}

// Hàm dừng động cơ
void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  delay(50);
}

// Hàm xoay trái cho đến khi không phát hiện tường phía trước
void rotateLeftUntilNoFrontWall() {
  Serial.println("Rotating left until no front wall detected...");
  unsigned long startTime = millis();
  
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 255);
  analogWrite(MOTOR_RIGHT_PWM, 255);

  while ((readSensor(SENSOR_0) > 200 || readSensor(SENSOR_3) > 200) && millis() - startTime < 5000) {
    delay(10);  // Tránh phản hồi nhanh
  }
  stopMotors();
  previousError = 0;  // Reset giá trị lỗi
}

// Hàm xoay phải cho đến khi không phát hiện tường phía trước
void rotateRightUntilNoFrontWall() {
  Serial.println("Rotating right until no front wall detected...");
  unsigned long startTime = millis();
  
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 255);
  analogWrite(MOTOR_RIGHT_PWM, 255);

  while ((readSensor(SENSOR_0) > 200 || readSensor(SENSOR_3) > 200) && millis() - startTime < 5000) {
    delay(10);
  }
  stopMotors();
  previousError = 0;  // Reset giá trị lỗi
}
// Hàm xoay tại chỗ
void rotateInPlace() {
  Serial.println("Rotating in place...");
  // Xoay trái: động cơ trái lùi, động cơ phải tiến
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(235 * 2);  // Thời gian xoay (cần chỉnh lại nếu cần)

  stopMotors();  // Dừng động cơ sau khi xoay
}
void moveOneCell() {
  Serial.println("Moving forward one cell...");
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(655);  // Điều chỉnh thời gian để di chuyển chính xác 1 ô
  stopMotors();
}

// Cập nhật trong hàm loop
// Cập nhật trong hàm loop
void loop() {
    float frontLeftSensor = readSensor(SENSOR_0);
    float frontRightSensor = readSensor(SENSOR_3);
    float leftSensor = readSensor(SENSOR_1);
    float rightSensor = readSensor(SENSOR_2);

    bool hasLeftWall = leftSensor > 100;
    bool hasRightWall = rightSensor > 100;
    bool hasFrontWall = (frontLeftSensor > 200 && frontRightSensor > 200);

    // Nếu không có tường trái, dừng xe, di chuyển 1 cell và rẽ trái
    if (leftSensor < 100) {
        Serial.println("No left wall, moving one cell and turning left...");
        stopMotors();
        moveOneCell();
        rotateLeftUntilNoFrontWall();
        // delay(50); // Đợi xe ổn định trước khi tiếp tục
    } else if (hasFrontWall) {
        stopMotors();
        if (hasLeftWall) {
            Serial.println("Rotating left...");
            rotateRightUntilNoFrontWall();
        } else if (hasRightWall) {
            Serial.println("Rotating right...");
            rotateLeftUntilNoFrontWall();
        } else {
            Serial.println("Rotating in place...");
            rotateInPlace();
        }
        // delay(20);  // Chờ ổn định sau khi xoay
    } else {
        float error = setPoint - leftSensor;
        float derivative = error - previousError;
        float correction = Kp * error + Kd * derivative;
        previousError = error;

        controlMotors(240, correction);
    }
  
    delay(10);
}


