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

void runBackward(unsigned int vl, unsigned int vr) {  
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
  motorSpeedControl(vl, vr);
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

void runForward(unsigned int vl, unsigned int vr) {
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
  motorSpeedControl(vl, vr);
#endif
  Serial.print("Motor Run : ");
  Serial.println(dir);
}

void leftTurn(unsigned int vl, unsigned int vr) {
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
  motorSpeedControl(vl, vr);
#endif
  Serial.print("Motor Left : ");
  Serial.println(dir);
}

void rightTurn(unsigned int vl, unsigned int vr) {
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
  motorSpeedControl(vl, vr);
#endif
  Serial.print("Motor Right : ");
  Serial.println(dir);
}

void motorSpeedControl(unsigned int vl, unsigned int vr) {
  int increase_millisec = 1000;
  for(int i = 0; i < increase_millisec; i++) {
    analogWrite(MOTOR_LEFT, vl * (i / increase_millisec));
    analogWrite(MOTOR_RIGHT, vr * (i / increase_millisec));
    delay(increase_millisec);
  }
}

void changeOperate() {
  if(mode == MODE_MANUAL) {
    leftSpeed = setLeftSpeed;
    rightSpeed = setRightSpeed;
  }

  switch (operate) {
    case OPERATE_FORWARD:
      runForward(leftSpeed,rightSpeed);
      state = STATE_FORWARD;
      break;
    case OPERATE_RIGHTTURN:
      rightTurn(leftSpeed,rightSpeed);
      state = STATE_RIGHTTURN;
      break;
    case OPERATE_LEFTTURN:
      leftTurn(leftSpeed,rightSpeed);
      state = STATE_LEFTTURN;
      break;
    case OPERATE_BACKWARD:
      runBackward(leftSpeed,rightSpeed);       
      state = STATE_BACKWARD;
      break;
    case OPERATE_STOP:
    default:
      Stop();
      state = STATE_STOP;
      break;  
  }
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
