// #include<Arduino.h>
// #include <Adafruit_MotorShield.h>
#include<AFMotor.h>
//#include<NewPing.h>

#define sensor1 A5 // phai1 S1
#define sensor2 A4 // phai2 S2
#define sensor3 A1 // trai2 S3
#define sensor4 A2 // trai1 S4
#define FOR(i,ii) for(int x = i ; x <= ii ; x++)

AF_DCMotor motor1(3, MOTOR12_1KHZ); // tren phai
AF_DCMotor motor2(4, MOTOR12_1KHZ); // tren trai
AF_DCMotor motor3(1, MOTOR34_1KHZ);// duoi trai
AF_DCMotor motor4(2, MOTOR34_1KHZ);// duoi phai


// Writer : Ninh Quang Dat

long distance , time;
const int trig = 2;  
const int echo = 13;
unsigned long init_time;

int count , tmp2 , tmp3 , times , huhu = 600;
int speedmax = 175 , speedmin = 0;
int sensor[5];
float P , I , D , PIDvalue;
float error , previousError;
float kP = 96.5, kI = 0 , kD = 4;
//float kP = 5.234286, kI = 0.00013 , kD = 0.0445;
int rsp , lsp;

void setup(){
  Serial.begin(9600);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
}

void readsensor(){
  sensor[1] = analogRead(sensor1);
  sensor[2] = analogRead(sensor2);
  sensor[3] = analogRead(sensor3);
  sensor[4] = analogRead(sensor4);
  //huhu = (sensor[1] + sensor[2] + sensor[3] + sensor[4]) / 4;
}

void  checksensor(){
  Serial.print(sensor[1]);Serial.print(" ");
  Serial.print(sensor[2]);Serial.print(" ");
  Serial.print(sensor[3]);Serial.print(" ");
  Serial.print(sensor[4]);Serial.println(" ");
  // Serial.print(P);Serial.print(" ");
  // Serial.print(I);Serial.print(" ");
  // Serial.print(D);Serial.print(" ");
  // Serial.print(PIDvalue);Serial.print(" ");
  // Serial.print(lsp);Serial.print(" ");
  // Serial.print(rsp);Serial.println(" ");
  // Serial.print(distance);Serial.println("    ");
  // Serial.print(init_time);Serial.print(" ");
  // Serial.print(fini_time);Serial.print(" ");
  // Serial.print(delta_time);Serial.println(" ");
  // Serial.print(count);Serial.println(" ");
  // Serial.print(times);Serial.print(" ");
  // Serial.print(time / 2);Serial.print(" ");
  // Serial.print(distance);Serial.println(" ");
  // delay(30);
   //Serial.print(distance);Serial.println("    ");
}

void caculate_error(){
  if( sensor[1] > huhu && sensor[2] < huhu && sensor[3] < huhu && sensor[4] < huhu ){ // 1 0 0 0 
    error = 3;
  }
  else if( sensor[1] > huhu && sensor[2] > huhu && sensor[3] < huhu && sensor[4] < huhu ){ // 1 1 0 0
    error = 2;
  }
  else if( sensor[1] < huhu && sensor[2] > huhu && sensor[3] < huhu && sensor[4] < huhu ){ // 0 1 0 0
     error = 1;
  }
  else if( sensor[1] < huhu && sensor[2] > huhu && sensor[3] > huhu && sensor[4] < huhu ){ // 0 1 1 0
     error = 0; 
  }
  else if( sensor[1] < huhu && sensor[2] < huhu && sensor[3] > huhu && sensor[4] < huhu ){ // 0 0 1 0
    error = -1; 
  }
  else if( sensor[1] < huhu && sensor[2] < huhu && sensor[3] > huhu && sensor[4] > huhu ){ // 0 0 1 1
     error = -2; 
  }
else if( sensor[1] < huhu && sensor[2] < huhu && sensor[3] < huhu && sensor[4] > huhu ){ // 0 0 0 1
     error = -3; 
  }
  else if( sensor[1] < huhu && sensor[2] < huhu && sensor[3] < huhu && sensor[4] < huhu ){  // 0 0 0 0
  if(error == -3) error = -4;
  else if(error == 3) error = 4;
  else error = 0;
  }
}

void caculate_PID(){
  P = error;
  I = I + error;
  D = error - previousError;
  previousError = error;
  PIDvalue = (kP * P) + (kI * I) + (kD * D);
  lsp = speedmax + PIDvalue;
  rsp = speedmax - PIDvalue;
  lsp = constrain(lsp, speedmin, speedmax);
  rsp = constrain(rsp, speedmin, speedmax);
}

long caculate_distance(){
  //distance = 0 ;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);  
  digitalWrite(trig, LOW);

  time = pulseIn(echo , HIGH);
  if(time > 10000) distance = 150;
  else distance = time / 58.824;
  delay(4);
  return distance;
}

void go(int ri , int le){
  motor1.run(FORWARD);
  motor1.setSpeed(ri);

  motor4.run(FORWARD);
  motor4.setSpeed(ri);

  motor2.run(FORWARD);
  motor2.setSpeed(le);

  motor3.run(FORWARD);
  motor3.setSpeed(le);

}

void left(int forward , int backward){
  motor1.run(FORWARD);
  motor1.setSpeed(forward);

  motor4.run(FORWARD);
  motor4.setSpeed(forward);

  motor2.run(BACKWARD);
  motor2.setSpeed(backward);

  motor3.run(BACKWARD);
  motor3.setSpeed(backward);
}

void right(int forward , int backward){
  motor2.run(FORWARD);
  motor2.setSpeed(forward);

  motor3.run(FORWARD);
  motor3.setSpeed(forward);

  motor1.run(BACKWARD);
  motor1.setSpeed(backward);

  motor4.run(BACKWARD);
  motor4.setSpeed(backward);
}

void rightright(){
    FOR(1, 500){
        tmp2 = analogRead(sensor2);
        tmp3 = analogRead(sensor3);
      if( tmp3 < huhu || tmp2 < huhu ){
        right(70,10);
      }
      else {
        return;
      }
    }
}

void leftleft(){
    FOR(1, 500){
        tmp2 = analogRead(sensor2);
        tmp3 = analogRead(sensor3);
      if( tmp2 < huhu || tmp3 < huhu){
        left(70,10);
      }
      else {
      return;
      }
    }
}

void curve(){
      FOR(1, 500){
        tmp2 = analogRead(sensor2);
        tmp3 = analogRead(sensor3);
      if( tmp2 < huhu || tmp3 < huhu ){
        right(80,10);
      }
      else {
        return;
      }
    }
}

void avoid_block(){
  constrain(rsp , 130 , 0);
  constrain(lsp , 130 , 0);
  unsigned long distmp = caculate_distance();
  times = 0;
  Serial.print(distance);Serial.println("    ");
  if(distmp < 17 ){
    go(0,0);delay(10);
    for(int i = 1 ; i <= 200 ; i++){
      unsigned long dis = caculate_distance();
      if(dis > 80){ 
      times = i;
      break;
      }
      else {
        right(255,255);delay(6);
        }
      }
    go(0,0); delay(25374);
    go(255,255); delay( times * 32);
    left(255,255);delay(times * 27);
    go(255 , 255);delay(times * 36);
    right(255,255);delay(times * 9);
  }
  else return;
}
/*************************************************************************************************************************************************************************/

// void follow_line(){
//   if( (count == 3 || count == 7 ) && sensor[3] > huhu && sensor[2] > huhu && sensor[4] > huhu){ // re trai
//       // go(0,0);delay(10);
//       count++;
//       left(200 , 160); delay(330);
//       leftleft();
//     }
//   else if(count == 6 && (sensor[4] < huhu && sensor[3] < huhu && sensor[2] < huhu && sensor[1] < huhu )){ // re trai truong hop dac biet
//     count++;
//     left(200 , 160); delay(300);
//     leftleft();
//     }
//   else if( ( count != 3 || count!= 7 ) && (sensor[2] > huhu && sensor[3] > huhu  && sensor[1] > huhu && sensor[4] < huhu) ){ // re phai
//    if(count == 2 || count == 4) {
//       count++;
//       right(200 , 150); delay(330);
//       rightright(); 
//     }
//     else {
//       count++;delay(400);
//     }
//   }
//   else if( count == 5 && (sensor[1] < 500 && sensor[2] < 500 && sensor[3] < 500 && sensor[4] < 500 ) ){ // gay
//     count++;
//     right(160 , 80); delay(700);
//     curve();
//   }
//   if(count == 8){  
//   avoid_block();
//   }
// init_time = millis();
// caculate_error();
// caculate_PID();
// go(rsp , lsp);
// }

/*************************************************************************************************************************************************************************/

void follow_line(){
  init_time = millis();
  if (sensor[3] > huhu && sensor[2] > huhu && init_time > 3300){
    right(200 , 160); delay(330);
    rightright(); 
  }
  else if( sensor[3] > huhu && sensor[2] > huhu && sensor[1] > huhu && ( init_time > 1370 && init_time < 1900 ) ) {
    right(200 , 160); delay(330);
    rightright(); 
  }
  else if(sensor[3] > huhu && sensor[2] > huhu && sensor[4] > huhu ){
      left(200 , 160); delay(300);
      leftleft();
  }
  else if(init_time > 6700 && sensor[1] < huhu && sensor[2] < huhu && sensor[3] < huhu && sensor[4] < huhu){
    right(200,120);delay(300);
    curve();  
  }
  else if(init_time > 7500 && sensor[1] < huhu && sensor[2] < huhu && sensor[3] < huhu && sensor[4] < huhu){
  left(200,160);delay(330);
  leftleft();
  }
  caculate_error();
  caculate_PID();
  go(rsp , lsp);
  if(init_time > 10000){go(0,0);delay(40000);}
}

/*************************************************************************************************************************************************************************/
void test(){ 
  avoid_block();
  caculate_error();
  caculate_PID();
  constrain(rsp , 80 , 0);
  constrain(lsp , 80 , 0);
  go(rsp , lsp);
}
void loop(){
  readsensor();

  follow_line();
  //test();
  //caculate_distance();
  checksensor();
}