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
