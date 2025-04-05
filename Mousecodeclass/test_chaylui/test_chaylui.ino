// Khai báo các chân
const uint8_t MOTOR_LEFT_DIR = 7;
const uint8_t MOTOR_RIGHT_DIR = 8;
const uint8_t MOTOR_LEFT_PWM = 9;
const uint8_t MOTOR_RIGHT_PWM = 10;
const uint8_t EMITTER_A = 11;
const uint8_t EMITTER_B = 12;
const uint8_t SENSOR_0 = A3;  // Cảm biến trước trái
const uint8_t SENSOR_3 = A0;  // Cảm biến trước phải
const uint8_t SENSOR_1 = A2;  // Cảm biến trái
const uint8_t SENSOR_2 = A1;  // Cảm biến phải

// Thông số PD
float Kp = 1.4;  // Hệ số tỷ lệ
float Kd = 0.9;  // Hệ số vi phân
int setPoint = 200;
float previousError = 0;

// Thời gian phát hiện vật cản trước
unsigned long obstacleStartTime = 0;
const unsigned long obstacleTimeout = 2000; // 2 giây

void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);
  pinMode(SENSOR_0, INPUT);
  pinMode(SENSOR_3, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);

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
  analogWrite(MOTOR_LEFT_PWM, leftSpeed);
  digitalWrite(MOTOR_RIGHT_DIR, LOW);
  analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_PWM, 0);
  analogWrite(MOTOR_RIGHT_PWM, 0);
}

void reverseMotors(int duration) {
  Serial.println("Reversing...");
  digitalWrite(MOTOR_LEFT_DIR, LOW);
  digitalWrite(MOTOR_RIGHT_DIR, HIGH);
  analogWrite(MOTOR_LEFT_PWM, 200);
  analogWrite(MOTOR_RIGHT_PWM, 200);
  delay(duration);
  stopMotors();
}

void loop() {
  float frontLeftSensor = readSensor(SENSOR_0);
  float frontRightSensor = readSensor(SENSOR_3);
  float leftSensor = readSensor(SENSOR_1);
  float rightSensor = readSensor(SENSOR_2);

  bool obstacleDetected = (frontLeftSensor > 100 && frontRightSensor > 100);

  // Kiểm tra thời gian xuất hiện vật cản
  if (obstacleDetected) {
    if (obstacleStartTime == 0) {
      obstacleStartTime = millis();
    }
    if (millis() - obstacleStartTime > obstacleTimeout) {
      reverseMotors(500);  // Lùi xe 0.5 giây
      obstacleStartTime = 0; // Reset thời gian
    }
  } else {
    obstacleStartTime = 0;
    float error = setPoint - leftSensor;
    float derivative = error - previousError;
    float correction = Kp * error + Kd * derivative;
    previousError = error;
    controlMotors(240, correction);
  }
  delay(10);
}
