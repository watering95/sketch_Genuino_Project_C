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

void motorBack(unsigned int vl, unsigned int vr) {  
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
  analogWrite(MOTOR_RIGHT, vr);
  analogWrite(MOTOR_LEFT, vl);
#endif
  Serial.print("Motor Back : ");
  Serial.println(dir);
}

void motorStop() {
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

void motorRun(unsigned int vl, unsigned int vr) {
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
  analogWrite(MOTOR_RIGHT, vr);
  analogWrite(MOTOR_LEFT, vl);
#endif
  Serial.print("Motor Run : ");
  Serial.println(dir);
}

void motorLeft(unsigned int vl, unsigned int vr) {
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
  analogWrite(MOTOR_LEFT, vl);
  analogWrite(MOTOR_RIGHT, vr);
#endif
  Serial.print("Motor Left : ");
  Serial.println(dir);
}

void motorRight(unsigned int vl, unsigned int vr) {
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
  analogWrite(MOTOR_LEFT, vl);
  analogWrite(MOTOR_RIGHT, vr);
#endif
  Serial.print("Motor Right : ");
  Serial.println(dir);
}

void changeRunState() {
  if(!isAuto) {
    leftSpeed = setLeftSpeed;
    rightSpeed = setRightSpeed;
  }
  Serial.print("left : ");
  Serial.print(leftSpeed);
  Serial.print(", ");
  Serial.print("right : ");
  Serial.println(rightSpeed);

  switch (motorState) {
    case MOTOR_STOP:
      motorStop();
      machineDirection = MACHINE_FORWARD;
      motorState = MOTOR_STOP;
      break;
    case MOTOR_RUN:
      if(isAdjusted || !isAuto) {
        motorRun(leftSpeed,rightSpeed);
        machineDirection = MACHINE_FORWARD;
        motorState = MOTOR_RUN;
      }
      break;
    case MACHINE_RIGHTTURN:
      motorRight(leftSpeed,rightSpeed);
      machineDirection = MACHINE_RIGHTTURN;
      motorState = MACHINE_RIGHTTURN;
      break;
    case MACHINE_LEFTTURN:
      motorLeft(leftSpeed,rightSpeed);
      machineDirection = MACHINE_LEFTTURN;
      motorState = MACHINE_LEFTTURN;
      break;
    case MACHINE_BACKWARD:
      if(isAdjusted || !isAuto) {
        motorBack(leftSpeed,rightSpeed);
        machineDirection = MACHINE_BACKWARD;
        motorState = MACHINE_BACKWARD;        
      }
      break;
    default:
      motorStop();
      machineDirection = MACHINE_FORWARD;
      motorState = MOTOR_STOP;
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



