#include "CurieBLE.h"
#include "CurieTimerOne.h"
#include "./MyMachine.h"
#include "./MyMotor.h"
#include "./MyIMU.h"

#define SHIELD_V1
//#define PROCESSING

unsigned int operate = OPERATE_STOP;
unsigned int state = STATE_STOP;
unsigned int mode = MODE_MANUAL;

const int adjustSpeed = 10;
const int angleRange = 1;
const int base = 30;

unsigned int now_vr = 0, now_vl = 0;

void setup() {
  Serial.begin(9600);
  initBLE();
  initMotorShield();
  prevTime = millis();
  initIMU();
  
  CurieTimerOne.start(timeBLE, &sendBLE); 

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  BLE.poll();

  nowTime = millis();
  dt = nowTime - prevTime;
  if(dt > timeIMU) {
    readIMU();

    angle_yaw = filtered_angle_yaw - base_yaw;
    angle_pitch = filtered_angle_pitch - base_pitch;
    angle_roll = filtered_angle_roll - base_roll;

    prevTime = nowTime;
  }
  if(mode == MODE_AUTO) autoRun();
  
#ifdef PROCESSING  
  sendToProcessing();
#endif
}

void autoRun() {
  decideDirection();
  decideSpeed();
  if(leftSpeed < 150) leftSpeed = 150;
  if(rightSpeed < 150) rightSpeed = 150;
  changeSpeed(0, leftSpeed, rightSpeed);
}

void decideDirection() {
  switch(operate) {
    case OPERATE_FORWARD:
      controlAngle = angle_yaw;
      runForward();
      state = STATE_FORWARD;
      break;
    case OPERATE_BACKWARD:
      controlAngle = angle_yaw;
      runBackward();
      state = STATE_BACKWARD;
      break;
    case OPERATE_LEFTTURN:
      controlAngle = angle_yaw + 90;
      leftTurn();
      state = STATE_LEFTTURN;
      break;
    case OPERATE_RIGHTTURN:
      controlAngle = angle_yaw - 90;
      rightTurn();
      state = STATE_RIGHTTURN;
      break;
    default:
      controlAngle = angle_yaw;
      state = STATE_STOP;
      break;
  }  
}

void decideSpeed() {
  if(controlAngle < angleRange) {
     if(state == OPERATE_STOP) {
       leftTurn();
     }
     else {
       leftSpeed = setLeftSpeed + adjustSpeed;
       rightSpeed = setRightSpeed - adjustSpeed;
     }
     isAdjusted = false;
  }
  else if(controlAngle > -angleRange) {
     if(state == OPERATE_STOP) {
       rightTurn();
     }
     else {
       leftSpeed = setLeftSpeed - adjustSpeed;
       rightSpeed = setRightSpeed + adjustSpeed;
     }
     isAdjusted = false;
  }
  else {
     if(state == OPERATE_STOP) {
       Stop();
     }
     else {
       leftSpeed = setLeftSpeed;
       rightSpeed = setRightSpeed;
     }
     isAdjusted = true;
  }
}

