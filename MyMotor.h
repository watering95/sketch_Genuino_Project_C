#ifdef SHIELD_V1
#define MOTOR_RIGHT 5  //H-Bridge M4
#define MOTOR_LEFT  6    //H-Bridge M3
#define MOTOR_ENABLE  7
#else
#define MOTOR_RIGHT 11  //H-Bridge M1
#define MOTOR_LEFT 3    //H-Bridge M2
#endif

#ifdef SHIELD_V2
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *motor2 = AFMS.getMotor(4);
#else
#define PIN_DATA 8
#define PIN_CLOCK 4
#define PIN_LATCH 12
#endif

void initMotorShield();
void manualOperate();
void changeSpeed(int, unsigned int, unsigned int);
void runBackward();
void runForward();
void rightTurn();
void leftTurn();
void Stop();

#ifndef SHIELD_V2
void changeDirection(byte);
#endif

