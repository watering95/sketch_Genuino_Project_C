#include "./MyMachine.h"

#define SHIELD_V1
//#define PROCESSING

unsigned int operate = OPERATE_STOP;
unsigned int new_operate = OPERATE_STOP;
unsigned int state = STATE_STOP;
unsigned int mode = MODE_MANUAL;

void setup() {
  Serial.begin(9600);
  initBLE();
  initMotorShield();
  prevPIDTime = millis();
  prevIMUTime = millis();
  initIMU();
  
  CurieTimerOne.start(timeBLE, &sendBLE); 

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  BLE.poll();

  nowTime = millis();
  dt_imu = nowTime - prevIMUTime;
  dt_pid = nowTime - prevPIDTime;
  
  if(dt_imu > timeIMU) {
    readIMU();
    angle_yaw = filtered_angle_yaw - base_yaw;
    prevIMUTime = nowTime;
  }
  if(dt_pid > timePID) {
    if(mode == MODE_AUTO) autoRun();   
    prevPIDTime = nowTime; 
  }
}

void autoRun() {
  if(new_operate != operate) {
    Stop();
    state = STATE_STOP;
    operate = new_operate;
    output = 0;
    iterm = 0;
    decideDirection();
  }
  
  stdPID(targetAngle, angle_yaw, prev_angle_yaw, kp, ki, kd, iterm, output);

  int leftSpeed, rightSpeed;
  
  if(operate == OPERATE_FORWARD) {
    leftSpeed = minimumSpeed + (maxSpeed - minimumSpeed) * (setLeftSpeed / 100.0) + (int)output;
    rightSpeed = minimumSpeed + (maxSpeed - minimumSpeed) * (setRightSpeed / 100.0) - (int)output;
  }
  else if(operate == OPERATE_BACKWARD) {
    leftSpeed = minimumSpeed + (maxSpeed - minimumSpeed) * (setLeftSpeed / 200.0) - (int)output;
    rightSpeed = minimumSpeed + (maxSpeed - minimumSpeed) * (setRightSpeed / 200.0) + (int)output;
  }
  else {
    leftSpeed = 0 -(int)output;
    rightSpeed = 0 + (int)output;
  }
  
  if(leftSpeed < 0 && rightSpeed > 0) {
    leftTurn();
  }
  else if(leftSpeed > 0 && rightSpeed < 0) {
    rightTurn();
  }
  else if(leftSpeed > 0 && rightSpeed > 0) {
    if(operate == OPERATE_FORWARD) {
      runForward();
    }
    else if(operate == OPERATE_BACKWARD) {
      runBackward();
    }
  }
  changeSpeed(abs(leftSpeed), abs(rightSpeed));
}

void decideDirection() {
  switch(operate) {
    case OPERATE_FORWARD:
      targetAngle = angle_yaw;
      state = STATE_FORWARD;
      break;
    case OPERATE_BACKWARD:
      targetAngle = angle_yaw;
      state = STATE_BACKWARD;
      break;
    case OPERATE_LEFTTURN:
      targetAngle = angle_yaw + 90;
      state = STATE_LEFTTURN;
      break;
    case OPERATE_RIGHTTURN:
      targetAngle = angle_yaw - 90;
      state = STATE_RIGHTTURN;
      break;
    case OPERATE_STOP:
    default:
      targetAngle = angle_yaw;
      state = STATE_STOP;
      break;
  }  
}

void stdPID(float& setpoint, float& input, float& prev_input, int& kp, int& ki, int& kd, float& iterm, float& output) {
  float error;
  float dInput;
  float pterm, dterm;

  error = setpoint - input;
  dInput = input - prev_input;
  prev_input = input;

  pterm = kp * error;
  iterm += ki * error * dt_pid;
  dterm = -kd * dInput / dt_pid;

  output = pterm + iterm + dterm;

  if(output < 0) {
    output = 0.1 * output - 150;
  }
  else {
    output = 0.1 * output + 150;
  }

  if(output > 250) output = 250;
  else if(output < -250) output = -250;
}

