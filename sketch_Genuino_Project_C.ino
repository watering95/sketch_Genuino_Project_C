#include <CurieBLE.h>
#include <CurieIMU.h>
#include "CurieTimerOne.h"

#define SHIELD_V1
//#define MADGWICK

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

#ifdef MADGWICK
#include <MadgwickAHRS.h>
Madgwick filter;
#endif

#define MACHINE_AUTO  1
#define MACHINE_MANUAL  0

#define MACHINE_FORWARD 2
#define MACHINE_BACKWARD  5
#define MACHINE_LEFTTURN  4
#define MACHINE_RIGHTTURN 3

#define MOTOR_STOP  0
#define MOTOR_RUN   1

BLEService machineService("BBB0");
BLECharacteristic machineStateChara("BBB1", BLERead | BLENotify, 30);

BLEService motorService("0174");
BLEIntCharacteristic stateChara("0175", BLEWrite);
BLEIntCharacteristic speedLeftChara("0176", BLEWrite);
BLEIntCharacteristic speedRightChara("0177", BLEWrite);
BLEIntCharacteristic isAutoChara("0178", BLEWrite);

boolean isConnectedCentral = false;
boolean isAdjusted = false;
boolean isAuto = false;

unsigned int leftSpeed = 0;
unsigned int rightSpeed = 0;
unsigned int setLeftSpeed = 0;
unsigned int setRightSpeed = 0;
unsigned int machineDirection = MACHINE_FORWARD;
unsigned int motorState = MOTOR_STOP;
char machineState[30];

const int timeBLE = 1000000;
const int timeIMU = 4;
const int adjustSpeed = 100;
const int range = 3;
const int base = 10;

int gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0; //RAW값
int baseGx = 0, baseGy = 0, baseGz = 0, baseAx = 0, baseAy = 0, baseAz = 0; //초기값


float filtered_angle_roll = 0;
float filtered_angle_pitch = 0;
float filtered_angle_yaw = 0;

int controlAngle;
int prevTime, nowTime;
float dt = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initBLE();
  initMotorShield();
  initIMU();

  CurieTimerOne.start(timeBLE, &sendBLE); 

  pinMode(LED_BUILTIN, OUTPUT);
  prevTime = millis();
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

void readIMU() {

#ifdef MADGWICK // Madgwick 사용 
  float accel_x = 0, accel_y = 0, accel_z = 0;
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  
  CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz);

  accel_x = convertRawAcceleration(ax);
  accel_y = convertRawAcceleration(ay);
  accel_z = convertRawAcceleration(az);

  gyro_x = convertRawGyro(gx);
  gyro_y = convertRawGyro(gy);
  gyro_z = convertRawGyro(gz);

  filter.updateIMU(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

  filtered_angle_roll = filter.getRoll();
  filtered_angle_pitch = filter.getPitch();
  filtered_angle_yaw = filter.getYaw();

#else // Madgwick 미사용

  const float RADIANS_TO_DEGREES = 180/3.14159;
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131.0;
  const float ALPHA = 0.96; // 입력주기 0.04, 시간상수 1
  
  int accel_x = 0, accel_y = 0, accel_z = 0;
  int gyro_x = 0, gyro_y = 0, gyro_z = 0;

  float accel_yz = 0, accel_xz = 0;

  float accel_angle_roll = 0;
  float accel_angle_pitch = 0;
  float accel_angle_yaw = 0;

  float gyro_angle_roll = 0;
  float gyro_angle_pitch = 0;
  float gyro_angle_yaw = 0;
  
  CurieIMU.readGyro(gx, gy, gz);
  CurieIMU.readAccelerometer(ax, ay, az); 

  accel_x = (ax - baseAx);  accel_y = (ay - baseAy) + 16384;  accel_z = (az - baseAz);

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_pitch = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_roll = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;
  
//  accel_angle_yaw = 0;

  gyro_x = (gx - baseGx) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (gy - baseGy) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (gz - baseGz) / GYROXYZ_TO_DEGREES_PER_SEC;

  gyro_angle_roll = gyro_x * dt / 1000;
  gyro_angle_pitch = gyro_y * dt / 1000;
  gyro_angle_yaw = gyro_z * dt / 1000;
  
  filtered_angle_roll = (ALPHA * (filtered_angle_roll + gyro_angle_roll)) + ((1.0 - ALPHA) * accel_angle_roll);
  filtered_angle_pitch = (ALPHA * (filtered_angle_pitch + gyro_angle_pitch)) + ((1.0 - ALPHA) * accel_angle_pitch);
  filtered_angle_yaw += gyro_angle_yaw;

  filtered_angle_roll = angle360(filtered_angle_roll);
  filtered_angle_pitch = angle360(filtered_angle_pitch);
  filtered_angle_yaw = angle360(filtered_angle_yaw);

  // data check
/*  
  String strAngleX = String(gyro_angle_roll, 3);
  String strAngleY = String(gyro_angle_pitch, 3);
  String strAngleZ = String(gyro_angle_yaw, 3);
*/
#endif
/*
  String strAngleX = String(gyro_x);
  String strAngleY = String(gyro_y);
  String strAngleZ = String(gyro_z);
*/
/*  
  String strAngleX = String(filtered_angle_roll, 3);
  String strAngleY = String(filtered_angle_pitch, 3);
  String strAngleZ = String(filtered_angle_yaw, 3);
*/

  String strAngleX = String(gyro_x);
  String strAngleY = String(gyro_angle_roll, 3);
  String strAngleZ = String(filtered_angle_roll, 3);
  String sendData = String(strAngleX + "," + strAngleY + "," + strAngleZ + ",");
  Serial.println(sendData);
}

float angle360(float angle) {
  if(angle < -360) angle += 360;
  else if(angle > 360) angle -= 360;
  return angle;
}

#ifdef MADGWICK
float convertRawAcceleration(int raw) {
  float a = (raw * 2.0) / 32768.0;
  return a;  
}

float convertRawGyro(int raw) {
  float g = (raw * 250.0) / 32768.0;
  return g;
}
#endif

void sendBLE() {
  if(!isConnectedCentral) return;
  
  String strState = String(motorState);
  String strAngleX = String(filtered_angle_roll, 1);
  String strAngleY = String(filtered_angle_pitch, 1);
  String strAngleZ = String(filtered_angle_yaw, 1);
  String sendData = String(strState + "," + strAngleX + "," + strAngleY + "," + strAngleZ + ",");
  sendData.toCharArray(machineState,sendData.length()+1);
//  Serial.println(machineState);
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
  Serial.println("Init Motor Shield V2");
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
#ifdef SHIELD_V2
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
#else
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1, M3 BWD, M4 BWD
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
#endif  
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, vr);
  analogWrite(MOTOR_LEFT, vl);
#endif
  Serial.print("Motor Back : ");
  Serial.println(dir);
  machineDirection = MACHINE_BACKWARD;
  motorState = MACHINE_BACKWARD;
}

void motorStop() {
#ifdef SHIELD_V2
  motor1->run(RELEASE);
  motor2->run(RELEASE);
#else
#ifdef SHIELD_V1
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0, M3 FWD, M4 FWD
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
#endif
  digitalWrite(LED_BUILTIN, LOW);  
  analogWrite(MOTOR_RIGHT, 0);
  analogWrite(MOTOR_LEFT, 0);
#endif
  Serial.print("Motor Stop : ");
  Serial.println(dir);
  machineDirection = MACHINE_FORWARD;
  motorState = MOTOR_STOP;
}

void motorRun(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(FORWARD);
  motor2->run(FORWARD);
#else
#ifdef SHIELD_V1
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0, M3 FWD, M4 FWD
#else
  byte dir = 0x05;  //  0 0 0 0 1 0 1 0
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, vr);
  analogWrite(MOTOR_LEFT, vl);
#endif
  Serial.print("Motor Run : ");
  Serial.println(dir);
  machineDirection = MACHINE_FORWARD;
  motorState = MOTOR_RUN;
}

void motorLeft(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(BACKWARD);
  motor2->run(RELEASE);
#else
#ifdef SHIELD_V1
  byte dir = 0x84;  //  1 0 0 0 0 1 0 0, M3 BWD, M4 FWD
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 0
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, vl);
  analogWrite(MOTOR_RIGHT, vr);
#endif
  Serial.print("Motor Left : ");
  Serial.println(dir);
  machineDirection = MACHINE_LEFTTURN;
  motorState = MACHINE_LEFTTURN;
}

void motorRight(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(RELEASE);
  motor2->run(BACKWARD);
#else
#ifdef SHIELD_V1
  byte dir = 0x03;  //  0 0 0 0 0 0 1 1, M3 FWD, M4 BWD
#else
  byte dir = 0x0A;  //  0 0 0 0 0 0 0 1
#endif
  digitalWrite(LED_BUILTIN, HIGH);  
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, vl);
  analogWrite(MOTOR_RIGHT, vr);
#endif
  Serial.print("Motor Right : ");
  Serial.println(dir);
  machineDirection = MACHINE_RIGHTTURN;
  motorState = MACHINE_RIGHTTURN;
}

void initBLE() {
  BLE.begin();
  
  BLE.setLocalName("GENUINO101");

  BLE.setAdvertisedService(machineService);
  machineService.addCharacteristic(machineStateChara);
  
  BLE.setAdvertisedService(motorService);
  motorService.addCharacteristic(stateChara);
  motorService.addCharacteristic(speedLeftChara);
  motorService.addCharacteristic(speedRightChara);
  motorService.addCharacteristic(isAutoChara);

  BLE.addService(machineService);
  BLE.addService(motorService);
  
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  stateChara.setEventHandler(BLEWritten, stateCharacteristicWritten);
  speedLeftChara.setEventHandler(BLEWritten, speedLeftCharacteristicWritten);
  speedRightChara.setEventHandler(BLEWritten, speedRightCharacteristicWritten);
  isAutoChara.setEventHandler(BLEWritten, isAutoCharacteristicWritten);
  
  BLE.advertise();
  Serial.println("BLE Genuino101 Peripheral");
}

void initIMU() {
  String data; 
  int tmpGx = 0, tmpGy = 0, tmpGz = 0, tmpAx = 0, tmpAy = 0, tmpAz = 0;
  
  Serial.println("Initializing IMU device..."); 
  CurieIMU.begin(); 
 
  CurieIMU.setGyroRate(25); 
  CurieIMU.setGyroRange(250); 

  #ifdef MADGWICK
  filter.begin(25);
  #endif
  
  CurieIMU.setAccelerometerRate(25); 
  CurieIMU.setAccelerometerRange(2); 

  for(int i=0; i < base; i++) {
    CurieIMU.readGyro(gx, gy, gz);
    CurieIMU.readAccelerometer(ax, ay, az);
    tmpGx += gx;    tmpGy += gy;    tmpGz += gz;
    tmpAx += ax;    tmpAy += ay;    tmpAz += az;
    data = String(String(tmpGx) + "," + String(tmpGy) + "," + String(tmpGz) + "," + String(tmpAx) + "," + String(tmpAy) + "," + String(tmpAz) + ",");
    Serial.println(data);
    delay(50);
  } 
/*// IMU 초기값 
  Serial.println("average");
  baseGx = tmpGx/base;  baseGy = tmpGy/base;  baseGz = tmpGz/base;
  baseAx = tmpAx/base;  baseAy = tmpAy/base;  baseAz = tmpAz/base;
  data = String(String(baseGx) + "," + String(baseGy) + "," + String(baseGz) + "," + String(baseAx) + "," + String(baseAy) + "," + String(baseAz) + ",");
  Serial.println(data);
*/
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
  Serial.println(setLeftSpeed);

  changeRunState();
}

void speedRightCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("speedRightCharacteristic event, written : ");
  setRightSpeed = speedRightChara.value();
  Serial.println(setRightSpeed);

  changeRunState();
}

void stateCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("directionCharacteristic event, written : ");
  motorState = stateChara.value();
  Serial.println(machineDirection);

  changeRunState();
}

void isAutoCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("isAutoCharacteristic event, written : ");
  if(isAutoChara.value() == MACHINE_AUTO) isAuto = true;
  else isAuto = false;
  Serial.println(isAuto);
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

void changeRunState() {
  if(!isAuto) {
    leftSpeed = setLeftSpeed;
    rightSpeed = setRightSpeed;
  }
  Serial.print("left : ");
  Serial.print(leftSpeed);
  Serial.print(", ");
  Serial.print("right : ");
  Serial.println(rightSpeed);

  switch (motorState) {
    case MOTOR_STOP:
      motorStop();
      break;
    case MOTOR_RUN:
      if(isAdjusted || !isAuto) {
        motorRun(leftSpeed,rightSpeed);
      }
      break;
    case MACHINE_RIGHTTURN:
      motorRight(leftSpeed,rightSpeed);
      break;
    case MACHINE_LEFTTURN:
      motorLeft(leftSpeed,rightSpeed);
      break;
    case MACHINE_BACKWARD:
      if(isAdjusted || !isAuto) {
        motorBack(leftSpeed,rightSpeed);        
      }
      break;
    default:
      motorStop();
      break;
  }
}

