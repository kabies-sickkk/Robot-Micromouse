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

// Thời gian di chuyển và quay
const unsigned long TURN_TIME_90_DEGREES = 237;  // ms
const unsigned long TURN_TIME_180_DEGREES = 440; // ms
const unsigned long TURN_TIME_CELL = 700;        // ms

// Thông số PD
float Kp = 0.27;  // Hệ số tỷ lệ
float Ki = 0.0;
float Kd = 0.05;  // Hệ số vi phân

int setPoint = 242; // Mục tiêu giá trị từ cảm biến trái
float previousError = 0;

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

float readSensor(uint8_t pin) {
  return analogRead(pin);
}

void controlMotors(int baseSpeed, float correction) {
  int leftSpeed = constrain(baseSpeed - correction, 0, 255);
  int rightSpeed = constrain(baseSpeed + correction, 0, 255);

  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, abs(leftSpeed));

  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, abs(rightSpeed));
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

void rotateRight() {
  Serial.println("Rotating 90 degrees to the right...");
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
}

void rotateLeft() {
  Serial.println("Rotating 90 degrees to the left...");
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
}

void moveOneCell() {
  Serial.println("Moving forward one cell...");
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_CELL);
  stopMotors();
}

void rotateInPlace() {
  Serial.println("Rotating 180 degrees...");
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_180_DEGREES);
  stopMotors();
}

void loop() {
  float frontLeftSensor = readSensor(SENSOR_0);
  float frontRightSensor = readSensor(SENSOR_3);
  float leftSensor = readSensor(SENSOR_1);
  float rightSensor = readSensor(SENSOR_2);

  bool hasLeftWall = leftSensor > 150;
  bool hasRightWall = rightSensor > 150;
  bool hasFrontWall = (frontLeftSensor > 100 && frontRightSensor > 100);
  
  bool hasNoLeftWall = leftSensor < 50;
  bool hasNoRightWall = rightSensor < 50;
  bool hasNoFrontWall = (frontLeftSensor < 50 && frontRightSensor < 50);

  if (hasFrontWall && hasLeftWall && hasRightWall) {
    rotateInPlace();
  } else if (hasNoLeftWall && hasNoRightWall && hasNoFrontWall) {
    moveOneCell();
    rotateLeft();
    moveOneCell();
  } else if (hasNoLeftWall && hasNoRightWall) {
    moveOneCell();
    rotateLeft();
  } else if (hasNoRightWall) {
    moveOneCell();
    rotateRight();
  } else if (hasNoLeftWall) {
    moveOneCell();
    rotateLeft();
  } else {
    float error = setPoint - leftSensor;
    float derivative = error - previousError;
    float correction = Kp * error + Kd * derivative;
    previousError = error;
    controlMotors(200, correction);
  }

  delay(10);
}
