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
BLEIntCharacteristic stateChara("0175", BLEWrite);
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

const int timeBLE = 1000000;
const int timeIMU = 100000;
const int adjustSpeed = 50;
const int range = 3;
const int base = 10;
const float RADIANS_TO_DEGREES = 180/3.14;
const float GYROXYZ_TO_DEGREES_PER_SEC = 131.0;
const float ALPHA = 0.96; // 입력주기 0.04, 시간상수 1

int gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
int baseGx = 0, baseGy = 0, baseGz = 0, baseAx = 0, baseAy = 0, baseAz = 0;
float filtered_angle_x = 0, filtered_angle_y = 0, filtered_angle_z = 0;
float gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;

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
  readIMU();
//  if(isAuto) autoRun();
//  delay(timeIMU);
}

void autoRun() {
  changeMotorAngle();
  if(controlAngle < range) {
     if(motorState == MOTOR_RUN) {
       motorLeftSpeed = setLeftSpeed + adjustSpeed;
       motorRightSpeed = setRightSpeed - adjustSpeed;
     }
     else {
       motorLeft(adjustSpeed, adjustSpeed);
     }
     isAdjusted = false;
  }
  else if(controlAngle > -range) {
     if(motorState == MOTOR_RUN) {
       motorLeftSpeed = setLeftSpeed - adjustSpeed;
       motorRightSpeed = setRightSpeed + adjustSpeed;
     }
     else {
       motorRight(adjustSpeed, adjustSpeed);
     }
     isAdjusted = false;
  }
  else {
     if(motorState == MOTOR_RUN) {
       motorLeftSpeed = setLeftSpeed;
       motorRightSpeed = setRightSpeed;
     }
     else {
       motorStop();
     }

    isAdjusted = true;
  }

  if(motorState == MOTOR_RUN) changeRunState();
}

void changeMotorAngle() {
  switch(motorDirection) {
    case MOTOR_FORWARD:
      controlAngle = filtered_angle_z;
      break;
    case MOTOR_BACKWARD:
      controlAngle = filtered_angle_z;
      break;
    case MOTOR_LEFTTURN:
      controlAngle = filtered_angle_z - 90;
      break;
    case MOTOR_RIGHTTURN:
      controlAngle = filtered_angle_z + 90;
      break;
  }  
  if(controlAngle < 0) controlAngle += 360;
  else if(controlAngle > 360) controlAngle -= 360;
}

void readIMU() {
  String data; 
  int accel_x = 0, accel_y = 0, accel_z = 0;
  int gyro_x = 0, gyro_y = 0, gyro_z = 0;
  float accel_yz = 0, accel_xz = 0;
  float accel_angle_x = 0, accel_angle_y = 0, accel_angle_z = 0;
 
  CurieIMU.readGyro(gx, gy, gz);
  CurieIMU.readAccelerometer(ax, ay, az); 
  nowTime = millis();
  dt = (nowTime - prevTime)/1000.0;
  prevTime = nowTime;

  accel_x = ax - baseAx;  accel_y = ay - baseAy;  accel_z = az + (16384 - baseAz);

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;
  
  accel_angle_z = 0;

  gyro_x = (gx - baseGx) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (gy - baseGy) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (gz - baseGz) / GYROXYZ_TO_DEGREES_PER_SEC;

  gyro_angle_x = gyro_x * dt;
  gyro_angle_y = gyro_y * dt;
  gyro_angle_z = gyro_z * dt;

  filtered_angle_x = ALPHA * (filtered_angle_x + gyro_angle_x) + (1.0 - ALPHA) * accel_angle_x;
  filtered_angle_y = ALPHA * (filtered_angle_y + gyro_angle_y) + (1.0 - ALPHA) * accel_angle_y;
  filtered_angle_z = ALPHA * (filtered_angle_z + gyro_angle_z) + (1.0 - ALPHA) * accel_angle_z;
}

void sendBLE() {
  if(!isConnectedCentral) return;
  
  String strState = String(motorState);
  String strAngleX = String(filtered_angle_x, 1);
  String strAngleY = String(filtered_angle_y, 1);
  String strAngleZ = String(filtered_angle_z, 1);
  String sendData = String(strState + "," + strAngleX + "," + strAngleY + "," + strAngleZ + ",");
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
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1, M3 BWD, M4 BWD
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
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0, M3 FWD, M4 FWD
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
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0, M3 FWD, M4 FWD
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
  byte dir = 0x03;  //  0 0 0 0 0 0 1 1, M3 FWD, M4 BWD
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
  byte dir = 0x84;  //  1 0 0 0 0 1 0 0, M3 BWD, M4 FWD
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
 
  CurieIMU.setGyroRate(100); 
  CurieIMU.setGyroRange(250); 
 
  CurieIMU.setAccelerometerRate(12.5); 
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
  Serial.println("average");
  baseGx = tmpGx/base;  baseGy = tmpGy/base;  baseGz = tmpGz/base;
  baseAx = tmpAx/base;  baseAy = tmpAy/base;  baseAz = tmpAz/base;
  data = String(String(baseGx) + "," + String(baseGy) + "," + String(baseGz) + "," + String(baseAx) + "," + String(baseAy) + "," + String(baseAz) + ",");
  Serial.println(data);
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

void stateCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("directionCharacteristic event, written : ");
  motorState = stateChara.value();
  Serial.println(motorDirection);

  changeRunState();
}

void isAutoCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("isAutoCharacteristic event, written : ");
  if(isAutoChara.value() == 1) isAuto = true;
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
