#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ============================================================
//  PIN ĐỊNH NGHĨA
// ============================================================
#define SS1 36
#define SS2 39
#define SS3 34
#define SS4 35
#define SS5 13
#define SENSOR_THRESHOLD 2500

#define PWB  17
#define BIN1 16
#define BIN2  4

#define PWA  21
#define AIN1 19
#define AIN2 18

#define ENCA_L 23
#define ENCB_L 22
#define ENCA_R 27
#define ENCB_R 26

#define trigPin 33
#define echoPin 25

#define PWM_FREQ 20000
#define PWM_RES  8

// ============================================================
//  THÔNG SỐ — điểm tốt nhất bạn tìm được
// ============================================================
int   BASE_SPEED = 115;

float Kp_line = 27.0;
float Ki_line =  0.01;
float Kd_line =  22.0;

float Kp_spd =  0.8;
float Ki_spd =  0.05;
float Kd_spd =  0.01;

#define OBSTACLE_DIST  15
#define AVOID_SPEED   150

// ============================================================
//  PROTOTYPE
// ============================================================
void setMotorL(int speed);
void setMotorR(int speed);
void motorStop(int t);
void motorForward(int sL, int sR, int t);
void motorBackward(int sL, int sR, int t);
void motorTurnLeft(int speed, int t);
void motorTurnRight(int speed, int t);
void motorDrive();
void readSensors();
void pidLineControl();
void pidSpeedBalance();
void readDistance();
bool obstacleConfirmed();
bool lineDetected();
void runAvoidObstacle();
void resetPidState();

// ============================================================
//  ENCODER
// ============================================================
volatile long encL = 0;
volatile long encR = 0;

void IRAM_ATTR isrEncL() {
  encL += (digitalRead(ENCB_L) == HIGH) ? 1 : -1;
}
void IRAM_ATTR isrEncR() {
  encR += (digitalRead(ENCB_R) == HIGH) ? -1 : 1;
}

// ============================================================
//  BIẾN TOÀN CỤC
// ============================================================
int   error = 0, last_error = 0;
int   left_speed = 0, right_speed = 0;
bool  lostLine = false;
long  duration;
int   distance;

float integLine = 0;

float integSpdL = 0, integSpdR = 0;
float prevErrSpdL = 0, prevErrSpdR = 0;
long  lastEncL = 0, lastEncR = 0;

#define SPD_INTERVAL 50

// ============================================================
//  BIẾN TRÁNH VẬT CẢN
// ============================================================
bool          isAvoiding      = false;
int           obstacleState   = 0;
unsigned long stateStartTime  = 0;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);

  pinMode(SS1, INPUT); pinMode(SS2, INPUT); pinMode(SS3, INPUT);
  pinMode(SS4, INPUT); pinMode(SS5, INPUT);

  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  ledcAttach(PWB, PWM_FREQ, PWM_RES);
  ledcAttach(PWA, PWM_FREQ, PWM_RES);

  pinMode(ENCA_L, INPUT_PULLUP); pinMode(ENCB_L, INPUT_PULLUP);
  pinMode(ENCA_R, INPUT_PULLUP); pinMode(ENCB_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), isrEncR, FALLING);

  motorStop(0);
  delay(2000);
  Serial.println("GO!");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  if (isAvoiding) {
    runAvoidObstacle();
    return;
  }

  if (obstacleConfirmed()) {
    motorStop(0);
    isAvoiding     = true;
    obstacleState  = 1;
    stateStartTime = millis();
    Serial.println("[AVOID] Phat hien vat can, bat dau tranh...");
    return;
  }

  readSensors();
  pidLineControl();
  pidSpeedBalance();
  motorDrive();
  delay(5);
}

// ============================================================
//  KIỂM TRA CÓ LINE KHÔNG
// ============================================================
bool lineDetected() {
  int s1 = (analogRead(SS1) > SENSOR_THRESHOLD) ? 1 : 0;
  int s2 = (analogRead(SS2) > SENSOR_THRESHOLD) ? 1 : 0;
  int s3 = (analogRead(SS3) > SENSOR_THRESHOLD) ? 1 : 0;
  int s4 = (analogRead(SS4) > SENSOR_THRESHOLD) ? 1 : 0;
  int s5 = (analogRead(SS5) > SENSOR_THRESHOLD) ? 1 : 0;
  return (s1 + s2 + s3 + s4 + s5) >= 2;
}

// ============================================================
//  STATE MACHINE TRÁNH VẬT CẢN
// ============================================================
void runAvoidObstacle() {
  unsigned long now = millis();

  if (obstacleState >= 5 && lineDetected()) {
    motorStop(0);
    Serial.print("[AVOID] Phat hien line tai state ");
    Serial.print(obstacleState);
    Serial.println(" -> nhay sang state 8 can chinh.");
    obstacleState  = 8;
    stateStartTime = millis();
    return;
  }

  switch (obstacleState) {

    case 1:
      motorBackward(AVOID_SPEED, AVOID_SPEED, 0);
      if (now - stateStartTime >= 300) {
        motorStop(0);
        stateStartTime = millis();
        obstacleState  = 2;
        Serial.println("[AVOID] S2: Quay trai");
      }
      break;

    case 2:
      motorTurnLeft(AVOID_SPEED, 0);
      if (now - stateStartTime >= 180) {
        motorStop(0);
        stateStartTime = millis();
        obstacleState  = 3;
        Serial.println("[AVOID] S3: Tien thang qua vat can");
      }
      break;

    case 3:
      motorForward(AVOID_SPEED, AVOID_SPEED, 0);
      if (now - stateStartTime >= 600) {
        motorStop(0);
        stateStartTime = millis();
        obstacleState  = 4;
        Serial.println("[AVOID] S4: Quay phai lan 1");
      }
      break;

    case 4:
      motorTurnRight(AVOID_SPEED, 0);
      if (now - stateStartTime >= 140) {
        motorStop(0);
        stateStartTime = millis();
        obstacleState  = 5;
        Serial.println("[AVOID] S5: Tien dai vuot qua vat can");
      }
      break;

    case 5:
      motorForward(AVOID_SPEED, AVOID_SPEED, 0);
      if (now - stateStartTime >= 850) {
        motorStop(0);
        stateStartTime = millis();
        obstacleState  = 6;
        Serial.println("[AVOID] S6: Quay phai lan 2");
      }
      break;

    case 6:
      motorTurnRight(AVOID_SPEED, 0);
      if (now - stateStartTime >= 200) {
        motorStop(0);
        stateStartTime = millis();
        obstacleState  = 7;
        Serial.println("[AVOID] S7: Tien tim vach");
      }
      break;

    case 7:
      motorForward(AVOID_SPEED, AVOID_SPEED, 0);
      if ((analogRead(SS3) > SENSOR_THRESHOLD) || (now - stateStartTime >= 1200)) {
        motorStop(0);
        stateStartTime = millis();
        obstacleState  = 8;
        if (analogRead(SS3) > SENSOR_THRESHOLD)
          Serial.println("[AVOID] S8: SS3 thay vach! Can chinh...");
        else
          Serial.println("[AVOID] S8: Het timeout, can chinh...");
      }
      break;

    case 8:
      motorTurnLeft(AVOID_SPEED, 0);
      if (analogRead(SS3) > SENSOR_THRESHOLD || now - stateStartTime >= 900) {
        motorStop(0);
        resetPidState();
        isAvoiding    = false;
        obstacleState = 0;
        Serial.println("[AVOID] Hoan thanh! Tiep tuc do line.");
      }
      break;

    default:
      isAvoiding    = false;
      obstacleState = 0;
      motorStop(0);
      break;
  }
}

// ============================================================
//  RESET PID SAU KHI TRÁNH VẬT CẢN
// ============================================================
void resetPidState() {
  error       = 0;
  last_error  = 0;
  integLine   = 0;
  integSpdL   = 0;
  integSpdR   = 0;
  prevErrSpdL = 0;
  prevErrSpdR = 0;
  lastEncL    = encL;
  lastEncR    = encR;
}

// ============================================================
//  ĐỌC CẢM BIẾN
// ============================================================
void readSensors() {
  int s1 = (analogRead(SS1) > SENSOR_THRESHOLD) ? 1 : 0;
  int s2 = (analogRead(SS2) > SENSOR_THRESHOLD) ? 1 : 0;
  int s3 = (analogRead(SS3) > SENSOR_THRESHOLD) ? 1 : 0;
  int s4 = (analogRead(SS4) > SENSOR_THRESHOLD) ? 1 : 0;
  int s5 = (analogRead(SS5) > SENSOR_THRESHOLD) ? 1 : 0;

  if      (s3==1 && s2==0 && s4==0)      { error =  0; lostLine = false; }
  else if (s3==1 && s4==1 && s2==0)      { error =  1; lostLine = false; }
  else if (s4==1 && s3==0 && s5==0)      { error =  2; lostLine = false; }
  else if (s4==1 && s5==1 && s3==0)      { error =  3; lostLine = false; }
  else if (s5==1 && s4==0)               { error =  4; lostLine = false; }
  else if (s3==1 && s2==1 && s4==0)      { error = -1; lostLine = false; }
  else if (s2==1 && s3==0 && s1==0)      { error = -2; lostLine = false; }
  else if (s2==1 && s1==1 && s3==0)      { error = -3; lostLine = false; }
  else if (s1==1 && s2==0)               { error = -4; lostLine = false; }
  else if (s1+s2+s3+s4+s5 >= 4)         { error =  0; lostLine = false; }
  else                                   { error = last_error; lostLine = true; }
}

// ============================================================
//  PID DÒ LINE — thuần, không filter, không leaky
// ============================================================
void pidLineControl() {
  float D    = (float)(error - last_error);
  last_error = error;

  integLine += (float)error;
  integLine  = constrain(integLine, -100, 100);

  float pid = Kp_line * (float)error
            + Ki_line * integLine
            + Kd_line * D;

  int dynSpeed;
  if      (abs(error) <= 1) dynSpeed = BASE_SPEED;
  else if (abs(error) == 2) dynSpeed = BASE_SPEED - 15;
  else if (abs(error) == 3) dynSpeed = BASE_SPEED - 30;
  else {
    if (error > 0) { left_speed = 240; right_speed = 0; }
    else           { left_speed = 0;   right_speed = 240; }
    integLine = 0;
    return;
  }

  left_speed  = constrain(dynSpeed + (int)pid, 0, 255);
  right_speed = constrain(dynSpeed - (int)pid, 0, 255);
}

// ============================================================
//  PID TỐC ĐỘ - CHỈ CHẠY KHI ĐI THẲNG
// ============================================================
void pidSpeedBalance() {
  if (abs(error) >= 1) return;

  static unsigned long lastSpd = 0;
  if (millis() - lastSpd < SPD_INTERVAL) return;
  lastSpd = millis();

  long l = encL, r = encR;
  float speedL = l - lastEncL;
  float speedR = r - lastEncR;
  lastEncL = l; lastEncR = r;

  float target = (speedL + speedR) / 2.0;

  float errL = target - speedL;
  float errR = target - speedR;

  integSpdL += errL; integSpdL = constrain(integSpdL, -300, 300);
  integSpdR += errR; integSpdR = constrain(integSpdR, -300, 300);

  float outL = Kp_spd * errL + Ki_spd * integSpdL + Kd_spd * (errL - prevErrSpdL);
  float outR = Kp_spd * errR + Ki_spd * integSpdR + Kd_spd * (errR - prevErrSpdR);

  prevErrSpdL = errL;
  prevErrSpdR = errR;

  left_speed  = constrain(left_speed  + (int)outL, 0, 255);
  right_speed = constrain(right_speed + (int)outR, 0, 255);
}

// ============================================================
//  ĐIỀU KHIỂN MOTOR
// ============================================================
void motorDrive() {
  setMotorL(left_speed);
  setMotorR(right_speed);
}

void setMotorL(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  ledcWrite(PWB, speed);  }
  else if (speed < 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); ledcWrite(PWB, -speed); }
  else                { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  ledcWrite(PWB, 0);      }
}

void setMotorR(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); ledcWrite(PWA, speed);  }
  else if (speed < 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  ledcWrite(PWA, -speed); }
  else                { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  ledcWrite(PWA, 0);      }
}

void motorStop(int t) {
  setMotorL(0); setMotorR(0);
  if (t > 0) delay(t);
}

void motorForward(int sL, int sR, int t) {
  setMotorL(sL); setMotorR(sR);
  if (t > 0) delay(t);
}

void motorBackward(int sL, int sR, int t) {
  setMotorL(-sL); setMotorR(-sR);
  if (t > 0) delay(t);
}

void motorTurnLeft(int speed, int t) {
  setMotorL(-speed); setMotorR(speed);
  if (t > 0) delay(t);
}

void motorTurnRight(int speed, int t) {
  setMotorL(speed); setMotorR(-speed);
  if (t > 0) delay(t);
}

// ============================================================
//  SIÊU ÂM
// ============================================================
void readDistance() {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH, 15000);
  distance = (duration == 0) ? 999 : (int)(duration * 0.034 / 2);
}

bool obstacleConfirmed() {
  int count = 0;
  for (int i = 0; i < 5; i++) {
    readDistance();
    if (distance >= 5 && distance <= OBSTACLE_DIST) count++;
    delay(10);
  }
  return count >= 4;
}