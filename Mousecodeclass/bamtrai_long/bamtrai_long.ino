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
const uint8_t SENSOR_0 = A3;
const uint8_t SENSOR_1 = A2; // Cảm biến trái
const uint8_t SENSOR_2 = A1;
const uint8_t SENSOR_3 = A0;
const int FUNCTION_PIN = A6;
const int BATTERY_VOLTS = A7;

// Khai báo thông số PID
float Kp = 1.9;  // Hệ số tỷ lệ
float Ki = 0.05; // Hệ số tích phân
float Kd = 0.015;  // Hệ số vi phân

int setPoint = 275; // Mục tiêu giá trị từ cảm biến
float previousError = 0;
float integral = 0;

// Hàm thiết lập
void setup() {
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(EMITTER_A, OUTPUT);
  pinMode(EMITTER_B, OUTPUT);
  pinMode(SENSOR_1, INPUT);
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

// Hàm vòng lặp chính
void loop() {
  // Đọc giá trị từ cảm biến
  float sensorValue = readSensor(SENSOR_1);
  
  // Tính toán PID
  float error = setPoint - sensorValue;
  // integral += error; // Tích lũy sai số
  float derivative = error - previousError; // Sai số tức thời
  float correction = Kp * error + Kd * derivative;

  previousError = error;

  // Gửi dữ liệu để debug
  Serial.print("Sensor: ");
  Serial.print(sensorValue);
  Serial.print(" Correction: ");
  Serial.println(correction);

  // Điều khiển động cơ với tốc độ cơ bản
  int baseSpeed = 200; // Tốc độ cơ bản của robot
  controlMotors(baseSpeed, correction);

  // Giới hạn tần số cập nhật
  delay(10);
}
