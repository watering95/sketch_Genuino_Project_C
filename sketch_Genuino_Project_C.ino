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

    angle_yaw = filtered_angle_yaw - base_yaw;
    angle_pitch = filtered_angle_pitch - base_pitch;
    angle_roll = filtered_angle_roll - base_roll;

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
      controlAngle = angle_yaw;
      break;
    case MACHINE_BACKWARD:
      controlAngle = angle_yaw;
      break;
    case MACHINE_LEFTTURN:
      controlAngle = angle_yaw - 90;
      break;
    case MACHINE_RIGHTTURN:
      controlAngle = angle_yaw + 90;
      break;
  }  
}
