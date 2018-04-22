#include "CurieBLE.h"
#include "CurieTimerOne.h"
#include "./MyMachine.h"
#include "./MyMotor.h"
#include "./MyIMU.h"

#define SHIELD_V1
//#define MADGWICK
#define PROCESSING

unsigned int machineDirection = MACHINE_FORWARD;
unsigned int motorState = MOTOR_STOP;

const int timeBLE = 1000000;
const int timeIMU = 200; //ms
const int adjustSpeed = 100;
const int range = 3;
const int base = 20;

int gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0; //RAW값
int baseGx = 0, baseGy = 0, baseGz = 0, baseAx = 0, baseAy = 0, baseAz = 0; //초기값

float filtered_angle_roll = 0;
float filtered_angle_pitch = 0;
float filtered_angle_yaw = 0;

float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0;

int controlAngle;
int prevTime = 0, nowTime = 0;
float dt = 0.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initBLE();
  initMotorShield();
  prevTime = millis();
  initIMU();

  CurieTimerOne.start(timeBLE, &sendBLE); 

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  BLE.poll();

  nowTime = millis();
  dt = nowTime - prevTime;
  if(dt > timeIMU) {
    readIMU();
    prevTime = nowTime;
  }
  if(isAuto) autoRun();
#ifdef PROCESSING  
  Serial.print("Orientation: ");
  Serial.print(angle_yaw);
  Serial.print(" ");
  Serial.print(angle_pitch);
  Serial.print(" ");
  Serial.println(angle_roll);
#endif
}

void autoRun() {
  changeMotorAngle();
  if(controlAngle < range) {
     if(motorState == MOTOR_RUN) {
       leftSpeed = setLeftSpeed + adjustSpeed;
       rightSpeed = setRightSpeed - adjustSpeed;
     }
     else {
       motorLeft(adjustSpeed, adjustSpeed);
     }
     isAdjusted = false;
  }
  else if(controlAngle > -range) {
     if(motorState == MOTOR_RUN) {
       leftSpeed = setLeftSpeed - adjustSpeed;
       rightSpeed = setRightSpeed + adjustSpeed;
     }
     else {
       motorRight(adjustSpeed, adjustSpeed);
     }
     isAdjusted = false;
  }
  else {
     if(motorState == MOTOR_RUN) {
       leftSpeed = setLeftSpeed;
       rightSpeed = setRightSpeed;
     }
     else {
       motorStop();
     }

    isAdjusted = true;
  }

  if(motorState == MOTOR_RUN) changeRunState();
}

void changeMotorAngle() {
  switch(machineDirection) {
    case MACHINE_FORWARD:
      controlAngle = filtered_angle_yaw;
      break;
    case MACHINE_BACKWARD:
      controlAngle = filtered_angle_yaw;
      break;
    case MACHINE_LEFTTURN:
      controlAngle = filtered_angle_yaw - 90;
      break;
    case MACHINE_RIGHTTURN:
      controlAngle = filtered_angle_yaw + 90;
      break;
  }  
  if(controlAngle < 0) controlAngle += 360;
  else if(controlAngle > 360) controlAngle -= 360;
}
