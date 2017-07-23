#include <CurieBLE.h>

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

BLEIntCharacteristic directionCharacteristic("BBB7", BLERead | BLEWrite);
BLEIntCharacteristic speedCharacteristic("BBB8", BLERead | BLEWrite);

BLEDescriptor directionDescriptor = BLEDescriptor("2901", "Direction");
BLEDescriptor speedDescriptor = BLEDescriptor("2901", "Speed");

boolean isConnectedCentral = false;
unsigned int motorDirection = 0;
unsigned int motorSpeed = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  initBLE();
  initMotorShield();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  blePeripheral.poll();
}

void initMotorShield() {
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);

  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
}

void motorRun(unsigned int v) {
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
  
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, v);
  analogWrite(MOTOR_LEFT, v);
}

void motorStop() {
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1

  digitalWrite(LED_BUILTIN, LOW);  
  analogWrite(MOTOR_RIGHT, 0);
  analogWrite(MOTOR_LEFT, 0);
}

void motorBack(unsigned int v) {
  byte dir = 0x05;  //  0 0 0 0 1 0 1 0

  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, v);
  analogWrite(MOTOR_LEFT, v);
}

void motorLeft(unsigned int v) {
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 0

  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, v);
  analogWrite(MOTOR_RIGHT, 0);
}

void motorRight(unsigned int v) {
  byte dir = 0x0A;  //  0 0 0 0 0 0 0 1
  
  digitalWrite(LED_BUILTIN, HIGH);  
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

void initBLE() {
  blePeripheral.setLocalName("GENUINO101");
  blePeripheral.setAdvertisedServiceUuid(genuinoService.uuid());

  blePeripheral.addAttribute(genuinoService);
  blePeripheral.addAttribute(directionCharacteristic);
  blePeripheral.addAttribute(directionDescriptor);
  blePeripheral.addAttribute(speedCharacteristic);
  blePeripheral.addAttribute(speedDescriptor);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  directionCharacteristic.setEventHandler(BLEWritten, directionCharacteristicWritten);
  speedCharacteristic.setEventHandler(BLEWritten, speedCharacteristicWritten);

  directionCharacteristic.setValue(0);
  speedCharacteristic.setValue(0);

  blePeripheral.begin();
  Serial.println("BLE Genuino101 Peripheral");
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
  Serial.println(motorSpeed);

  changeRunState();
}

void directionCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print("directionCharacteristic event, written : ");
  motorDirection = directionCharacteristic.value();
  Serial.println(motorDirection);

  changeRunState();
}

void changeRunState() {
  switch (motorDirection) {
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
