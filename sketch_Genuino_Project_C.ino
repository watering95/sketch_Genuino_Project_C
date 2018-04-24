#include "CurieBLE.h"
#include "CurieTimerOne.h"
#include "./MyMachine.h"
#include "./MyMotor.h"
#include "./MyIMU.h"

#define SHIELD_V1
//#define PROCESSING

unsigned int operate = OPERATE_FORWARD;
unsigned int state = STATE_STOP;
unsigned int mode = MODE_MANUAL;

const int adjustSpeed = 100;
const int angleRange = 3;
const int base = 30;

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
  changeMotorAngle();
  if(controlAngle < angleRange) {
     if(state == STATE_STOP) {
       leftTurn(adjustSpeed, adjustSpeed);
     }
     else {
       leftSpeed = setLeftSpeed + adjustSpeed;
       rightSpeed = setRightSpeed - adjustSpeed;
     }
     isAdjusted = false;
  }
  else if(controlAngle > -angleRange) {
     if(state == STATE_STOP) {
       rightTurn(adjustSpeed, adjustSpeed);
     }
     else {
       leftSpeed = setLeftSpeed - adjustSpeed;
       rightSpeed = setRightSpeed + adjustSpeed;
     }
     isAdjusted = false;
  }
  else {
     if(state == STATE_STOP) {
       Stop();
     }
     else {
       leftSpeed = setLeftSpeed;
       rightSpeed = setRightSpeed;
     }

    isAdjusted = true;
  }

  if(state != STATE_STOP) changeOperate();
}

void changeMotorAngle() {
  switch(operate) {
    case OPERATE_FORWARD:
      controlAngle = angle_yaw;
      break;
    case OPERATE_BACKWARD:
      controlAngle = angle_yaw;
      break;
    case OPERATE_LEFTTURN:
      controlAngle = angle_yaw - 90;
      break;
    case OPERATE_RIGHTTURN:
      controlAngle = angle_yaw + 90;
      break;
  }  
}
