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
float Kp = 0.27;  // Hệ số tỷ lệ
float Kd = 0.05;  // Hệ số vi phân

int setPoint = 150;  // Mục tiêu giá trị từ cảm biến trái
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
  delay(100);
}

// Hàm xoay 90 độ sang phải
void rotateRight() {
  Serial.println("Rotating 90 degrees to the right...");
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(237);
  stopMotors();
}

// Hàm xoay 90 độ sang trái
void rotateLeft() {
  Serial.println("Rotating 90 degrees to the left...");
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(237);
  stopMotors();
}
// Hàm xoay tại chỗ
void rotateInPlace() {
  Serial.println("Rotating in place...");
  // Xoay trái: động cơ trái lùi, động cơ phải tiến
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(225 * 2);  // Thời gian xoay (cần chỉnh lại nếu cần)

  stopMotors();  // Dừng động cơ sau khi xoay
}
void moveOneCell() {
  Serial.println("Moving forward one cell...");
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(655);
  stopMotors();
}

// Hàm vòng lặp chính
void loop() {
  float frontLeftSensor = readSensor(SENSOR_0);
  float frontRightSensor = readSensor(SENSOR_3);
  float leftSensor = readSensor(SENSOR_1);
  float rightSensor = readSensor(SENSOR_2);

  bool hasLeftWall = leftSensor > 100;
  bool hasRightWall = rightSensor > 100;
  bool hasFrontWall = (frontLeftSensor > 100 && frontRightSensor > 100);

  if (hasFrontWall) {
    stopMotors();
    if (hasLeftWall) {
      Serial.println("Front wall and left wall detected.");
      rotateRight();
    } else if (hasRightWall) {
      Serial.println("Front wall and right wall detected.");
      rotateLeft();
    } else if (hasLeftWall && hasRightWall) {
      Serial.println("All walls detected: Front, Left, Right.");
      rotateInPlace();  // Xoay tại chỗ
    }
    return;
  }
  // Kiểm tra điều kiện có tường trước, tường trái và tường phải
  // if (hasFrontWall && hasLeftWall && hasRightWall) {
  //   Serial.println("All walls detected: Front, Left, Right.");
  //   stopMotors();
  //   rotateInPlace();  // Xoay tại chỗ
  //   return;           // Quay lại vòng lặp chính
  // }
  // if (SENSOR_1 < 50){
  //   stopMotors();
  //   moveOneCell();
  //   rotateLeft();
  //   return;
  // }

  float error = setPoint - leftSensor;
  float derivative = error - previousError;
  float correction = Kp * error + Kd * derivative;

  previousError = error;

  Serial.print("Left Sensor: ");
  Serial.print(leftSensor);
  Serial.print(", Front Left: ");
  Serial.print(frontLeftSensor);
  Serial.print(", Front Right: ");
  Serial.print(frontRightSensor);
  Serial.print(", Right Sensor: ");
  Serial.println(rightSensor);

  int baseSpeed = 240;
  controlMotors(baseSpeed, correction);

  delay(10);
}
