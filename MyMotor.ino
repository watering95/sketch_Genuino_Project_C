#include "MyMotor.h"

void initMotorShield() {
#ifdef SHIELD_V1
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, LOW);
#endif
#ifdef SHIELD_V2  
  AFMS.begin();
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  Serial.println("Init Motor Shield V2");
#else
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);

  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
  Serial.println("Init Motor Shield V1");
#endif
}

void changeSpeed(int change_millisec, unsigned int vl, unsigned int vr) {
  unsigned int change_vr = vr - now_vr;
  unsigned int change_vl = vl - now_vl;
  
  if(operate == OPERATE_STOP) {
    now_vr = 0;
    now_vl = 0;
    return;
  }
  
  for(int i = 1, limit = change_millisec + 1; i < limit; i++) {
    analogWrite(MOTOR_LEFT, now_vl + (change_vl * i) / limit);
    analogWrite(MOTOR_RIGHT, now_vr + (change_vr * i) / limit);
    delay(change_millisec*10);
  }
  now_vr = vr;
  now_vl = vl;
}

void changeDirection(byte d) {
  digitalWrite(PIN_LATCH, LOW);
  Serial.println("Shift Register Latch Low");
  shiftOut(PIN_DATA, PIN_CLOCK, LSBFIRST, d);
  Serial.println(d);
  digitalWrite(PIN_LATCH, HIGH);
  Serial.println("Shift Register Latch High (Hold)");
  delay(500);
}

void manualOperate() {
  switch (operate) {
    case OPERATE_FORWARD:
      runForward();
      state = STATE_FORWARD;
      break;
    case OPERATE_RIGHTTURN:
      rightTurn();
      state = STATE_RIGHTTURN;
      break;
    case OPERATE_LEFTTURN:
      leftTurn();
      state = STATE_LEFTTURN;
      break;
    case OPERATE_BACKWARD:
      runBackward();       
      state = STATE_BACKWARD;
      break;
    case OPERATE_STOP:
    default:
      Stop();
      state = STATE_STOP;
      break;  
  }
}

void runBackward() {  
#ifdef SHIELD_V2
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
#else
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1, M3 BWD, M4 BWD
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
#endif  
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
#endif
  Serial.print("Motor Back : ");
  Serial.println(dir);
}

void Stop() {
#ifdef SHIELD_V2
  motor1->run(RELEASE);
  motor2->run(RELEASE);
#else
#ifdef SHIELD_V1
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0, M3 FWD, M4 FWD
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
#endif
  digitalWrite(LED_BUILTIN, LOW);  
  analogWrite(MOTOR_RIGHT, 0);
  analogWrite(MOTOR_LEFT, 0);
#endif
  Serial.print("Motor Stop : ");
  Serial.println(dir);
}

void runForward() {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(FORWARD);
  motor2->run(FORWARD);
#else
#ifdef SHIELD_V1
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0, M3 FWD, M4 FWD
#else
  byte dir = 0x05;  //  0 0 0 0 1 0 1 0
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
#endif
  Serial.print("Motor Run : ");
  Serial.println(dir);
}

void leftTurn() {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(BACKWARD);
  motor2->run(RELEASE);
#else
#ifdef SHIELD_V1
  byte dir = 0x84;  //  1 0 0 0 0 1 0 0, M3 BWD, M4 FWD
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 0
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
#endif
  Serial.print("Motor Left : ");
  Serial.println(dir);
}

void rightTurn() {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(RELEASE);
  motor2->run(BACKWARD);
#else
#ifdef SHIELD_V1
  byte dir = 0x03;  //  0 0 0 0 0 0 1 1, M3 FWD, M4 BWD
#else
  byte dir = 0x0A;  //  0 0 0 0 0 0 0 1
#endif
  digitalWrite(LED_BUILTIN, HIGH);  
  changeDirection(dir);
#endif
  Serial.print("Motor Right : ");
  Serial.println(dir);
}
