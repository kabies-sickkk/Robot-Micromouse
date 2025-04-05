#include <Arduino.h>

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

const unsigned long TURN_TIME_90_DEGREES = 237;  // ms
const unsigned long TURN_TIME_180_DEGREES = 450; // ms
const unsigned long TURN_TIME_CELL = 655;        // ms

float Kp = 0.27;
float Ki = 0.0;
float Kd = 0.05;

int setPoint = 242;
float previousError = 0;

enum RobotState {
  FOLLOW_WALL,
  SEARCH_LEFT_WALL,
  TURN_LEFT,
  TURN_RIGHT,
  MOVE_ONE_CELL,
  TURN_AROUND
};

RobotState currentState = FOLLOW_WALL;

// Ngưỡng cho cảm biến
const int THRESHOLD_LEFT_WALL = 150;
const int THRESHOLD_RIGHT_WALL = 150;
const int THRESHOLD_FRONT_WALL = 100;
const int NO_WALL_THRESHOLD = 50;

// Khởi tạo
void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);
  digitalWrite(EMITTER_A, HIGH);
  digitalWrite(EMITTER_B, HIGH);
  Serial.begin(115200);
}

// Đọc giá trị cảm biến
float readSensor(uint8_t pin) {
  return analogRead(pin);
}

// Điều khiển động cơ
void controlMotors(int baseSpeed, float correction) {
  int leftSpeed = constrain(baseSpeed - correction, 0, 255);
  int rightSpeed = constrain(baseSpeed + correction, 0, 255);
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, leftSpeed);
  analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
}

// Các thao tác di chuyển
void moveOneCell() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_CELL);
  stopMotors();
}

void rotateLeft() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
}

void rotateRight() {
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_90_DEGREES);
  stopMotors();
}

void rotateInPlace() {
  digitalWrite(MOTOR_LEFT_DIR, HIGH);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(TURN_TIME_180_DEGREES);
  stopMotors();
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

// Hàm xử lý các trạng thái
void handleState(float leftSensor, float rightSensor, float frontLeftSensor, float frontRightSensor) {
  bool hasLeftWall = leftSensor > THRESHOLD_LEFT_WALL;
  bool hasFrontWall = (frontLeftSensor > THRESHOLD_FRONT_WALL && frontRightSensor > THRESHOLD_FRONT_WALL);
  bool hasNoLeftWall = leftSensor < NO_WALL_THRESHOLD;

  switch (currentState) {
    case FOLLOW_WALL:
      if (hasFrontWall) {
        currentState = TURN_AROUND;
      } else if (hasNoLeftWall) {
        currentState = SEARCH_LEFT_WALL;
      } else {
        float error = setPoint - leftSensor;
        float derivative = error - previousError;
        float correction = Kp * error + Kd * derivative;
        previousError = error;
        controlMotors(200, correction);
      }
      break;

    case SEARCH_LEFT_WALL:
      moveOneCell();
      if (hasNoLeftWall) {
        rotateLeft();
        moveOneCell();
      }
      if (hasNoLeftWall) {
        currentState = SEARCH_LEFT_WALL; // Tiếp tục tìm kiếm
      } else {
        currentState = FOLLOW_WALL;
      }
      break;

    case TURN_LEFT:
      rotateLeft();
      moveOneCell();
      currentState = FOLLOW_WALL;
      break;

    case TURN_RIGHT:
      rotateRight();
      currentState = FOLLOW_WALL;
      break;

    case TURN_AROUND:
      rotateInPlace();
      currentState = FOLLOW_WALL;
      break;
  }
}

// Loop chính
void loop() {
  float frontLeftSensor = readSensor(SENSOR_0);
  float frontRightSensor = readSensor(SENSOR_3);
  float leftSensor = readSensor(SENSOR_1);
  float rightSensor = readSensor(SENSOR_2);

  handleState(leftSensor, rightSensor, frontLeftSensor, frontRightSensor);

  delay(10);
}
