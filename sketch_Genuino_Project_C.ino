#include <CurieBLE.h>
#include "CurieIMU.h"

#define AVERAGE 5
#define AXIS_X  0
#define AXIS_Y  1
#define AXIS_Z  2

#define PIN_DATA 8
#define PIN_CLOCK 4
#define PIN_LATCH 12

#define MOTOR_RIGHT 11  //H-Bridge M1
#define MOTOR_LEFT 3    //H-Bridge M2

#define MACHINE_FORWARD 1
#define MACHINE_BACKWARD  4
#define MACHINE_LEFT  3
#define MACHINE_RIGHT 2
#define MACHINE_STOP  0

BLEPeripheral blePeripheral;
BLEService genuinoService("BBB0");
BLEFloatCharacteristic gyroXChar("BBB1", BLERead | BLENotify);
BLEFloatCharacteristic gyroYChar("BBB2", BLERead | BLENotify);
BLEFloatCharacteristic gyroZChar("BBB3", BLERead | BLENotify);
BLEFloatCharacteristic acclXChar("BBB4", BLERead | BLENotify);
BLEFloatCharacteristic acclYChar("BBB5", BLERead | BLENotify);
BLEFloatCharacteristic acclZChar("BBB6", BLERead | BLENotify);

BLEIntCharacteristic directionCharacteristic("BBB7", BLERead | BLEWrite);
BLEIntCharacteristic speedCharacteristic("BBB8", BLERead | BLEWrite);

BLEDescriptor gyroXDescriptor = BLEDescriptor("2901", "gyroX");
BLEDescriptor gyroYDescriptor = BLEDescriptor("2901", "gyroY");
BLEDescriptor gyroZDescriptor = BLEDescriptor("2901", "gyroZ");
BLEDescriptor acclXDescriptor = BLEDescriptor("2901", "acclX");
BLEDescriptor acclYDescriptor = BLEDescriptor("2901", "acclY");
BLEDescriptor acclZDescriptor = BLEDescriptor("2901", "acclZ");

BLEDescriptor directionDescriptor = BLEDescriptor("2901", "Direction");
BLEDescriptor speedDescriptor = BLEDescriptor("2901", "Speed");

unsigned long movetime[2] = {0, 0};
float init_a[3], g[3][AVERAGE], a[3][AVERAGE];
float avg_g[3][2], avg_a[3][2];
boolean isInitialized = false;
boolean isConnectedCentral = false;
unsigned int motorDirection = 0;
unsigned int motorSpeed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(19200);
  initIMU();
  initBLE();
  initMotorShield();
}

void loop() {
  static int index = 0;
  unsigned long millisec = 0;
  float sec;

  // put your main code here, to run repeatedly:
  blePeripheral.poll();

  CurieIMU.readGyroScaled(g[AXIS_X][index], g[AXIS_Y][index], g[AXIS_Z][index]);
  CurieIMU.readAccelerometerScaled(a[AXIS_X][index], a[AXIS_Y][index], a[AXIS_Z][index]);

  for(int axis = 0; axis < 3; axis++) {  
    if(!isInitialized) {
      avg_a[axis][0] = filter(a[axis], index + 1);
      if(index == AVERAGE - 1) {
        if(axis == AXIS_Z) isInitialized = true;
      }
      else {
        avg_a[axis][0] = filter(a[axis], AVERAGE);
      } 
    }
  }

  movetime[1] = millis();
  millisec = movetime[1] - movetime[0];
  sec = millisec / 1000.0;

  if (isConnectedCentral) sendBLE(index);

  if (index < AVERAGE - 1) index++;
  else index = 0;

  movetime[0] = movetime[1];
}

void initMotorShield() {
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);

  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
}

void motorRun(unsigned int v) {
  byte dir = 0x05;  //  0 0 0 0 0 1 0 1
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, v);
  analogWrite(MOTOR_LEFT, v);
}

void motorStop() {
  byte dir = 0x05;  //  0 0 0 0 0 1 0 1
  analogWrite(MOTOR_RIGHT, 0);
  analogWrite(MOTOR_LEFT, 0);
}

void motorBack(unsigned int v) {
  byte dir = 0x0A;  //  0 0 0 0 1 0 1 0
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, v);
  analogWrite(MOTOR_LEFT, v);
}

void motorLeft(unsigned int v) {
  byte dir = 0x04;  //  0 0 0 0 0 1 0 0
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, v);
  analogWrite(MOTOR_RIGHT, 0);
}

void motorRight(unsigned int v) {
  byte dir = 0x01;  //  0 0 0 0 0 0 0 1
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, 0);
  analogWrite(MOTOR_RIGHT, v);
}

void changeDirection(byte d) {
  digitalWrite(PIN_LATCH, LOW);
  shiftOut(PIN_DATA, PIN_CLOCK, MSBFIRST, d);
  digitalWrite(PIN_LATCH, HIGH);
  delay(500);
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

void initBLE() {
  blePeripheral.setLocalName("GENUINO101");
  blePeripheral.setAdvertisedServiceUuid(genuinoService.uuid());

  blePeripheral.addAttribute(genuinoService);
  blePeripheral.addAttribute(gyroXChar);
  blePeripheral.addAttribute(gyroXDescriptor);
  blePeripheral.addAttribute(gyroYChar);
  blePeripheral.addAttribute(gyroYDescriptor);
  blePeripheral.addAttribute(gyroZChar);
  blePeripheral.addAttribute(gyroZDescriptor);
  blePeripheral.addAttribute(acclXChar);
  blePeripheral.addAttribute(acclXDescriptor);
  blePeripheral.addAttribute(acclYChar);
  blePeripheral.addAttribute(acclYDescriptor);
  blePeripheral.addAttribute(acclZChar);
  blePeripheral.addAttribute(acclZDescriptor);
  blePeripheral.addAttribute(directionCharacteristic);
  blePeripheral.addAttribute(directionDescriptor);
  blePeripheral.addAttribute(speedCharacteristic);
  blePeripheral.addAttribute(speedDescriptor);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  directionCharacteristic.setEventHandler(BLEWritten, directionCharacteristicWritten);
  speedCharacteristic.setEventHandler(BLEWritten, speedCharacteristicWritten);

  gyroXChar.setValue(0);  gyroYChar.setValue(0);  gyroZChar.setValue(0);
  acclXChar.setValue(0);  acclYChar.setValue(0);  acclZChar.setValue(0);

  directionCharacteristic.setValue(0);
  speedCharacteristic.setValue(0);
  
  blePeripheral.begin();
  Serial.println("BLE Genuino101 Peripheral");
}

void sendBLE(int index) {
  gyroXChar.setValue(g[AXIS_X][index]);
  gyroYChar.setValue(g[AXIS_Y][index]);
  gyroZChar.setValue(g[AXIS_Z][index]);
  acclXChar.setValue(a[AXIS_X][index]);
  acclYChar.setValue(a[AXIS_Y][index]);
  acclZChar.setValue(a[AXIS_Z][index]);
}

void blePeripheralConnectHandler(BLECentral& central) {
  Serial.print("Connected event, central : ");
  Serial.println(central.address());
  isConnectedCentral = true;
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  Serial.print("Disconnected event, central : ");
  Serial.println(central.address());
  isConnectedCentral = false;
}

void speedCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print("speedCharacteristic event, written : ");
  motorSpeed = speedCharacteristic.value();

  changeRunState();
}

void directionCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print("directionCharacteristic event, written : ");
  motorDirection = directionCharacteristic.value();
  Serial.println(motorDirection);  
      
  changeRunState();
}

void changeRunState() {
  switch(motorDirection) {
    case MACHINE_STOP:
      motorStop();
      break;
    case MACHINE_FORWARD:
      motorRun(motorSpeed);
      break;
    case MACHINE_RIGHT:
      motorRight(motorSpeed);
      break;
    case MACHINE_LEFT:
      motorLeft(motorSpeed);
      break;
    case MACHINE_BACKWARD:
      motorBack(motorSpeed);
      break;   
    default:
      motorStop();
      break;
    }  
}

float Integration(float base, float diff0, float diff1, float t) {
  float result = 0.0;
  result = base + (diff0 + ((diff1 - diff0) / 2.0)) * t;

  return result;
}

float filter(float raw[], int num) {
  float result = 0.0;
  for (int i = 0; i < num; i++) {
    result = result + raw[i];
  }

  return result / num;
}
