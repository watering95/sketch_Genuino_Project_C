#include "CurieBLE.h"
#include "CurieTimerOne.h"
#include "./MyMachine.h"
#include "./MyMotor.h"
#include "./MyIMU.h"

#define SHIELD_V1
//#define PROCESSING

unsigned int operate = OPERATE_STOP;
unsigned int new_operate = OPERATE_STOP;
unsigned int state = STATE_STOP;
unsigned int mode = MODE_MANUAL;

const int adjustSpeed = 50;
const int angleRange = 10;
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
  if(new_operate != operate) {
    operate = new_operate;
    decideDirection();
  }
  decideSpeed();
  if(leftSpeed < 20) leftSpeed = 20;
  if(rightSpeed < 20) rightSpeed = 20;
  changeSpeed(3, leftSpeed, rightSpeed);
  if(operate == OPERATE_STOP || operate == OPERATE_LEFTTURN || operate == OPERATE_RIGHTTURN) {
    delay(5);
    Stop();
    delay(200);
  }
}

void decideDirection() {
  switch(operate) {
    case OPERATE_FORWARD:
      targetAngle = angle_yaw;
      runForward();
      state = STATE_FORWARD;
      break;
    case OPERATE_BACKWARD:
      targetAngle = angle_yaw;
      runBackward();
      state = STATE_BACKWARD;
      break;
    case OPERATE_LEFTTURN:
      targetAngle = angle_yaw + 90;
      leftTurn();
      state = STATE_LEFTTURN;
      break;
    case OPERATE_RIGHTTURN:
      targetAngle = angle_yaw - 90;
      rightTurn();
      state = STATE_RIGHTTURN;
      break;
    default:
      targetAngle = angle_yaw;
      state = STATE_STOP;
      break;
  }  
}

void decideSpeed() {
  float left_limit = angle360(targetAngle + angleRange);
  float right_limit = angle360(targetAngle - angleRange);

  switch(operate) {
    case OPERATE_STOP:
      if(angle_yaw > left_limit) {
        if(state != STATE_RIGHTTURN) {
          rightTurn();
          state = STATE_RIGHTTURN;
        }
      }
      else if(angle_yaw > right_limit) {
        if(state != STATE_RIGHTTURN) {
          rightTurn();
          state = STATE_RIGHTTURN;
        }
      }
      else {
        if(state != STATE_STOP) {
          Stop();
          state = STATE_STOP;
        }
      }
      break;
    case OPERATE_FORWARD:
      if(angle_yaw < left_limit) {
        leftSpeed = setLeftSpeed + adjustSpeed;
        rightSpeed = setRightSpeed - adjustSpeed;
      }
      else if(angle_yaw > right_limit) {
        leftSpeed = setLeftSpeed - adjustSpeed;
        rightSpeed = setRightSpeed + adjustSpeed;
      }
      else {
        leftSpeed = setLeftSpeed;
        rightSpeed = setRightSpeed;
      }
      break;
    case OPERATE_BACKWARD:
      if(angle_yaw < left_limit) {
        leftSpeed = setLeftSpeed - adjustSpeed;
        rightSpeed = setRightSpeed + adjustSpeed;
      }
      else if(angle_yaw > right_limit) {
        leftSpeed = setLeftSpeed + adjustSpeed;
        rightSpeed = setRightSpeed - adjustSpeed;
      }
      else {
        leftSpeed = setLeftSpeed;
        rightSpeed = setRightSpeed;
      }
      break;
    case OPERATE_LEFTTURN:
      if(angle_yaw > left_limit) {
        Stop();
        state = STATE_STOP;
        new_operate = OPERATE_STOP;
      }
      else {
        leftSpeed = setLeftSpeed;
        rightSpeed = setRightSpeed;
      }
      break;
    case OPERATE_RIGHTTURN:
      if(angle_yaw < right_limit) {
        Stop();
        state = STATE_STOP;
          new_operate = OPERATE_STOP;
      }
      else {
        leftSpeed = setLeftSpeed;
        rightSpeed = setRightSpeed;
      }
      break;
  }
}

