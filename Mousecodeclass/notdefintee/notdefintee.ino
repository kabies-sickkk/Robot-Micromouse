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
const uint8_t SENSOR_0 = A3; // Cảm biến trước trái
const uint8_t SENSOR_1 = A2; // Cảm biến trái
const uint8_t SENSOR_2 = A1; // Cảm biến phải
const uint8_t SENSOR_3 = A0; // Cảm biến trước phải
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;

// Thông số PD
float Kp = 2.0;  // Hệ số tỷ lệ
float Kd = 0.025; // Hệ số vi phân

int setPoint = 265; // Mục tiêu giá trị từ cảm biến trái
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

  Serial.begin(115200); // Dùng để debug
}

// Hàm đọc giá trị cảm biến
float readSensor(uint8_t pin) {
  return analogRead(pin);
}

// Hàm điều khiển động cơ
void controlMotors(int baseSpeed, float correction) {
  int leftSpeed = constrain(baseSpeed - correction, 0, 255); // Bánh trái bù trừ ngược
  int rightSpeed = constrain(baseSpeed + correction, 0, 255); // Bánh phải bù trừ cùng chiều

  // Điều khiển chiều và tốc độ động cơ trái
  digitalWrite(MOTOR_LEFT_DIR, HIGH);  // Luôn tiến tới
  analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed));

  // Điều khiển chiều và tốc độ động cơ phải
  digitalWrite(MOTOR_RIGHT_DIR, LOW); // Luôn tiến tới
  analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed));
}

// Hàm dừng động cơ
void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  Serial.println("Motors stopped.");
}

// Hàm xoay tại chỗ
void rotateInPlace() {
  Serial.println("Rotating in place...");
  // Xoay trái: động cơ trái lùi, động cơ phải tiến
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(440); // Thời gian xoay (cần chỉnh lại nếu cần)

  stopMotors(); // Dừng động cơ sau khi xoay
}

// Hàm vòng lặp chính
void loop() {
  // Đọc giá trị từ các cảm biến
  float frontLeftSensor = readSensor(SENSOR_0);  // Cảm biến trước trái
  float frontRightSensor = readSensor(SENSOR_3); // Cảm biến trước phải
  float leftSensor = readSensor(SENSOR_1);       // Cảm biến trái
  float rightSensor = readSensor(SENSOR_2);      // Cảm biến phải

  // Kiểm tra điều kiện có tường trước, tường trái và tường phải
  bool hasLeftWall = leftSensor > 250;
  bool hasRightWall = rightSensor > 250;
  bool hasFrontWall = (frontLeftSensor > 100 && frontRightSensor > 100);

  if (hasFrontWall && hasLeftWall && hasRightWall) {
    Serial.println("All walls detected: Front, Left, Right.");
    stopMotors();
    rotateInPlace(); // Xoay tại chỗ
    return; // Quay lại vòng lặp chính
  }

  // Tính toán PD để bám tường trái
  float error = setPoint - leftSensor; // Sai số so với mục tiêu
  float derivative = error - previousError; // Sai số tức thời
  float correction = Kp * error + Kd * derivative; // Tính toán hiệu chỉnh

  previousError = error;

  // Debug thông tin cảm biến
  Serial.print("Left Sensor: ");
  Serial.print(leftSensor);
  Serial.print(", Front Left: ");
  Serial.print(frontLeftSensor);
  Serial.print(", Front Right: ");
  Serial.print(frontRightSensor);
  Serial.print(", Right Sensor: ");
  Serial.println(rightSensor);

  // Điều khiển động cơ với tốc độ cơ bản
  int baseSpeed = 200; // Tốc độ cơ bản
  controlMotors(baseSpeed, correction);

  // Giới hạn tần số cập nhật
  delay(10);
}
