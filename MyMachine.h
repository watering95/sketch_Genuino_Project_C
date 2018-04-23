#define MACHINE_AUTO  1
#define MACHINE_MANUAL  0

#define MACHINE_FORWARD 2
#define MACHINE_BACKWARD  5
#define MACHINE_LEFTTURN  4
#define MACHINE_RIGHTTURN 3

unsigned int leftSpeed = 0;
unsigned int rightSpeed = 0;
unsigned int setLeftSpeed = 0;
unsigned int setRightSpeed = 0;

boolean isConnectedCentral = false;
boolean isAdjusted = false;
boolean isAuto = false;

const int timeBLE = 1000000;
const int timeIMU = 50; //ms
const int adjustSpeed = 100;
const int range = 3;
const int base = 30;

int gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0; //RAW값
int base_ax = 0, base_ay = 0, base_az = 0, base_gx = 0, base_gy = 0, base_gz = 0;
float base_roll = 0, base_pitch = 0, base_yaw = 0; //초기값

float filtered_angle_roll = 0;
float filtered_angle_pitch = 0;
float filtered_angle_yaw = 0;

float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0;

int controlAngle;
int prevTime = 0, nowTime = 0;
float dt = 0.0;
