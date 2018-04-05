#include <CurieBLE.h>
#include "CurieIMU.h"
#include "CurieTimerOne.h"

#define SHIELD_V1

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

#define MOTOR_FORWARD 2
#define MOTOR_BACKWARD  5
#define MOTOR_LEFTTURN  4
#define MOTOR_RIGHTTURN 3
#define MOTOR_STOP  0
#define MOTOR_RUN   1

BLEService machineService("BBB0");
BLECharacteristic machineStateChara("BBB1", BLERead | BLENotify, 30);

BLEService motorService("0174");
BLEIntCharacteristic directionChara("0175", BLEWrite);
BLEIntCharacteristic speedLeftChara("0176", BLEWrite);
BLEIntCharacteristic speedRightChara("0177", BLEWrite);
BLEIntCharacteristic isAutoChara("0178", BLEWrite);

boolean isConnectedCentral = false;
boolean isAdjusted = false;
boolean isAuto = false;
unsigned int motorLeftSpeed = 0;
unsigned int motorRightSpeed = 0;
unsigned int setLeftSpeed = 0;
unsigned int setRightSpeed = 0;
unsigned int motorDirection = MOTOR_FORWARD;
unsigned int motorState = MOTOR_STOP;
char machineState[30];
const int timeBLE = 1000;
const int adjustSpeed = 50;

float gx, gy, gz, ax, ay, az;
int controlDirection;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initBLE();
  initMotorShield();
  initIMU();

  CurieTimerOne.start(timeBLE, &sendBLE); 
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  BLE.poll();
  readIMU();
  if(isAuto) autoRun();
}

void autoRun() {
  changeMotorDirection();
  if(controlDirection < 0) {
     motorLeftSpeed = adjustSpeed;
     motorRightSpeed = 0;
     isAdjusted = false;
  }
  else if(controlDirection > 0) {
     motorLeftSpeed = 0;
     motorRightSpeed = adjustSpeed;
     isAdjusted = false;
  }
  else {
    motorLeftSpeed = 0;
    motorRightSpeed = 0;
    isAdjusted = true;
  }
  if(motorState != MOTOR_STOP) {
    motorLeftSpeed += setLeftSpeed;
    motorRightSpeed += setRightSpeed;
  }
  changeRunState();
}

void changeMotorDirection() {
  switch(motorDirection) {
    case MOTOR_FORWARD:
      controlDirection = ay;
      break;
    case MOTOR_BACKWARD:
      controlDirection = ay;
      break;
    case MOTOR_LEFTTURN:
      controlDirection = ax;
      break;
    case MOTOR_RIGHTTURN:
      controlDirection = ax;
      break;
  }   
}

void readIMU() {
  CurieIMU.readGyroScaled(gx, gy, gz);
  CurieIMU.readAccelerometerScaled(ax, ay, az); 
}

void sendBLE() {
  if(!isConnectedCentral) return;
  
  String strState = String(motorState);
  String strAx = String(ax, 1);
  String strAy = String(ay, 1);
  String strAz = String(az, 1);
  String strGx = String(gx, 1);
  String strGy = String(gy, 1);
  String strGz = String(gz, 1);
  String sendData = String(strState + "," + strAx + "," + strAy + "," + strAz + "," + strGx + "," + strGy + "," + strGz + ",");
  sendData.toCharArray(machineState,sendData.length()+1);
  Serial.println(machineState);
  machineStateChara.setValue(machineState);
}

void initMotorShield() {
#ifdef SHIELD_V1
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, LOW);
#endif
#ifdef SHIELD_V2  
  AFMS.begin();
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  Serial.println("Init Motor Shield V2);
#else
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);

  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
  Serial.println("Init Motor Shield V1");
#endif
}

void motorBack(unsigned int vl, unsigned int vr) {  
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1
#else
#ifdef SHIELD_V2
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
#endif  
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, vr);
  analogWrite(MOTOR_LEFT, vl);
#endif
  Serial.print("Motor Run : ");
  Serial.println(dir);
  motorDirection = MOTOR_BACKWARD;
  motorState = MOTOR_BACKWARD;
}

void motorStop() {
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1
#else
#ifdef SHIELD_V2
  motor1->run(RELEASE);
  motor2->run(RELEASE);
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
#endif
  digitalWrite(LED_BUILTIN, LOW);  
  analogWrite(MOTOR_RIGHT, 0);
  analogWrite(MOTOR_LEFT, 0);
#endif
  Serial.print("Motor Stop : ");
  Serial.println(dir);
  motorDirection = MOTOR_FORWARD;
  motorState = MOTOR_STOP;
}

void motorRun(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V1
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0
#else
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(FORWARD);
  motor2->run(FORWARD);
#else
  byte dir = 0x05;  //  0 0 0 0 1 0 1 0
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, vr);
  analogWrite(MOTOR_LEFT, vl);
#endif
  Serial.print("Motor Back : ");
  Serial.println(dir);
  motorDirection = MOTOR_FORWARD;
  motorState = MOTOR_RUN;
}

void motorLeft(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V1
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0
#else
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(BACKWARD);
  motor2->run(RELEASE);
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 0
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, vl);
  analogWrite(MOTOR_RIGHT, 0);
#endif
  Serial.print("Motor Left : ");
  Serial.println(dir);
  motorDirection = MOTOR_LEFTTURN;
  motorState = MOTOR_LEFTTURN;
}

void motorRight(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V1
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0
#else
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(RELEASE);
  motor2->run(BACKWARD);
#else
  byte dir = 0x0A;  //  0 0 0 0 0 0 0 1
#endif
  digitalWrite(LED_BUILTIN, HIGH);  
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, 0);
  analogWrite(MOTOR_RIGHT, vr);
#endif
  Serial.print("Motor Right : ");
  Serial.println(dir);
  motorDirection = MOTOR_RIGHTTURN;
  motorState = MOTOR_RIGHTTURN;
}

#ifndef SHIELD_V2
void changeDirection(byte d) {
  digitalWrite(PIN_LATCH, LOW);
  Serial.println("Shift Register Latch Low");
  shiftOut(PIN_DATA, PIN_CLOCK, LSBFIRST, d);
  Serial.println(d);
  digitalWrite(PIN_LATCH, HIGH);
  Serial.println("Shift Register Latch High (Hold)");
  delay(500);
}
#endif

void initBLE() {
  BLE.begin();
  
  BLE.setLocalName("GENUINO101");

  BLE.setAdvertisedService(machineService);
  machineService.addCharacteristic(machineStateChara);
  
  BLE.setAdvertisedService(motorService);
  motorService.addCharacteristic(directionChara);
  motorService.addCharacteristic(speedLeftChara);
  motorService.addCharacteristic(speedRightChara);
  motorService.addCharacteristic(isAutoChara);

  BLE.addService(machineService);
  BLE.addService(motorService);
  
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  directionChara.setEventHandler(BLEWritten, directionCharacteristicWritten);
  speedLeftChara.setEventHandler(BLEWritten, speedLeftCharacteristicWritten);
  speedRightChara.setEventHandler(BLEWritten, speedRightCharacteristicWritten);
  isAutoChara.setEventHandler(BLEWritten, isAutoCharacteristicWritten);
  
  BLE.advertise();
  Serial.println("BLE Genuino101 Peripheral");
}

void initIMU() { 
  Serial.println("Initializing IMU device..."); 
  CurieIMU.begin(); 
 
  CurieIMU.setGyroRate(100); 
  CurieIMU.setGyroRange(250); 
 
  CurieIMU.setAccelerometerRate(12.5); 
  CurieIMU.setAccelerometerRange(2); 
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0); 
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0); 
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);  
} 

void blePeripheralConnectHandler(BLEDevice central) {
  Serial.print("Connected event, central : ");
  Serial.println(central.address());
  isConnectedCentral = true;
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  Serial.print("Disconnected event, central : ");
  Serial.println(central.address());
  isConnectedCentral = false;
}

void speedLeftCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("speedLeftCharacteristic event, written : ");
  setLeftSpeed = speedLeftChara.value();
  Serial.println(motorLeftSpeed);

  changeRunState();
}

void speedRightCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("speedRightCharacteristic event, written : ");
  setRightSpeed = speedRightChara.value();
  Serial.println(motorRightSpeed);

  changeRunState();
}

void directionCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("directionCharacteristic event, written : ");
  motorDirection = directionChara.value();
  Serial.println(motorDirection);

  changeRunState();
}

void isAutoCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("isAutoCharacteristic event, written : ");
  if(directionChara.value() == 1) isAuto = true;
  else isAuto = false;
}

void changeRunState() {
  switch (motorState) {
    case MOTOR_STOP:
      motorStop();
      break;
    case MOTOR_RUN:
      if(isAdjusted || !isAuto) {
        motorRun(motorLeftSpeed,motorRightSpeed);
      }
      break;
    case MOTOR_RIGHTTURN:
      motorRight(motorLeftSpeed,motorRightSpeed);
      break;
    case MOTOR_LEFTTURN:
      motorLeft(motorLeftSpeed,motorRightSpeed);
      break;
    case MOTOR_BACKWARD:
      if(isAdjusted || !isAuto) {
        motorBack(motorLeftSpeed,motorRightSpeed);        
      }
      break;
    default:
      motorStop();
      break;
  }
}
