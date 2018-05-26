#include "CurieTimerOne.h"
#include "./MyIMU.h"
#include "./MyBLE.h"

#define MODE_AUTO  1
#define MODE_MANUAL  0

#define OPERATE_FORWARD 2
#define OPERATE_BACKWARD  5
#define OPERATE_LEFTTURN  4
#define OPERATE_RIGHTTURN 3
#define OPERATE_STOP  0

#define STATE_STOP  0
#define STATE_FORWARD   2
#define STATE_BACKWARD  5
#define STATE_LEFTTURN  4
#define STATE_RIGHTTURN 3

int setLeftSpeed = 0;
int setRightSpeed = 0;

int kp = 3, ki = 0, kd = 1;

int prevPIDTime = 0, prevIMUTime = 0, nowTime = 0;

int gx = 0, gy = 0, gz = 0;
int base_gz = 0;

const int minimumSpeed = 150;
const int maxSpeed = 250;
const int base = 30;

const int timeBLE = 2000000;
const int timeIMU = 100; //ms
const int timePID = 1; //ms

boolean isConnectedCentral = false;
boolean isAdjusted = false;

float base_yaw = 0; //초기값
float filtered_angle_yaw = 0;
float angle_yaw = 0;

float targetAngle;

float prev_angle_yaw = 0.0, iterm, output;
float dt_imu = 0.0;
float dt_pid = 0.0;
