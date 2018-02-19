#include <CurieBLE.h>

#define SHIELD_V1

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

#ifdef SHIELD_V1
#define MOTOR_RIGHT 5  //H-Bridge M4
#define MOTOR_LEFT  6    //H-Bridge M3
#define MOTOR_ENABLE  7
#else
#define MOTOR_RIGHT 11  //H-Bridge M1
#define MOTOR_LEFT 3    //H-Bridge M2
#endif

#define MACHINE_FORWARD 1
#define MACHINE_BACKWARD  4
#define MACHINE_LEFT  3
#define MACHINE_RIGHT 2
#define MACHINE_STOP  0

BLEPeripheral blePeripheral;
BLEService genuinoService("BBB0");

BLEIntCharacteristic directionCharacteristic("BBB7", BLERead | BLEWrite);
BLEIntCharacteristic speedLeftCharacteristic("BBB8", BLERead | BLEWrite);
BLEIntCharacteristic speedRightCharacteristic("BBB9", BLERead | BLEWrite);

BLEDescriptor directionDescriptor = BLEDescriptor("2901", "Direction");
BLEDescriptor speedLeftDescriptor = BLEDescriptor("2901", "SpeedLeft");
BLEDescriptor speedRightDescriptor = BLEDescriptor("2901", "SpeedRight");

boolean isConnectedCentral = false;
unsigned int motorDirection = 0;
unsigned int motorLeftSpeed = 0;
unsigned int motorRightSpeed = 0;

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
#ifdef SHIELD_V2  
  AFMS.begin();
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  Serial.println("Init Motor Shield V2);
#else
#ifdef SHIELD_V1
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, LOW);
#endif
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);

  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
  Serial.println("Init Motor");
#endif
}

void motorRun(unsigned int vl, unsigned int vr) {  
#ifdef SHIELD_V2
  motor1->run(FORWARD);
  motor2->run(FORWARD);
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
#else
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
#endif  
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, vr);
  analogWrite(MOTOR_LEFT, vl);
  Serial.print("Motor Run : ");
  Serial.println(dir);
#endif
}

void motorStop() {
#ifdef SHIELD_V2
  motor1->run(RELEASE);
  motor2->run(RELEASE);
#else
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 1
#endif
  digitalWrite(LED_BUILTIN, LOW);  
  analogWrite(MOTOR_RIGHT, 0);
  analogWrite(MOTOR_LEFT, 0);
  Serial.print("Motor Stop : ");
  Serial.println(dir);
#endif
}

void motorBack(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(BACKWARD);
  motor2->run(BACKWARD);
#else
#ifdef SHIELD_V1
  byte dir = 0x06;  //  0 0 0 0 0 1 1 0
#else
  byte dir = 0x05;  //  0 0 0 0 1 0 1 0
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, vr);
  analogWrite(MOTOR_LEFT, vl);
  Serial.print("Motor Back : ");
  Serial.println(dir);
#endif
}

void motorLeft(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(BACKWARD);
  motor2->run(RELEASE);
#else
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1
#else
  byte dir = 0x0A;  //  0 0 0 0 0 1 0 0
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, vl);
  analogWrite(MOTOR_RIGHT, 0);
  Serial.print("Motor Left : ");
  Serial.println(dir);
#endif
}

void motorRight(unsigned int vl, unsigned int vr) {
#ifdef SHIELD_V2
  motor1->setSpeed(vl);
  motor2->setSpeed(vr);
  motor1->run(RELEASE);
  motor2->run(BACKWARD);
#else
#ifdef SHIELD_V1
  byte dir = 0x81;  //  1 0 0 0 0 0 0 1
#else
  byte dir = 0x0A;  //  0 0 0 0 0 0 0 1
#endif
  digitalWrite(LED_BUILTIN, HIGH);  
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, 0);
  analogWrite(MOTOR_RIGHT, vr);
  Serial.print("Motor Right : ");
  Serial.println(dir);
#endif
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
  blePeripheral.setLocalName("GENUINO101");
  blePeripheral.setAdvertisedServiceUuid(genuinoService.uuid());

  blePeripheral.addAttribute(genuinoService);
  blePeripheral.addAttribute(directionCharacteristic);
  blePeripheral.addAttribute(directionDescriptor);
  blePeripheral.addAttribute(speedLeftCharacteristic);
  blePeripheral.addAttribute(speedLeftDescriptor);
  blePeripheral.addAttribute(speedRightCharacteristic);
  blePeripheral.addAttribute(speedRightDescriptor);

  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  directionCharacteristic.setEventHandler(BLEWritten, directionCharacteristicWritten);
  speedLeftCharacteristic.setEventHandler(BLEWritten, speedLeftCharacteristicWritten);
  speedRightCharacteristic.setEventHandler(BLEWritten, speedRightCharacteristicWritten);

  directionCharacteristic.setValue(0);
  speedLeftCharacteristic.setValue(0);
  speedRightCharacteristic.setValue(0);

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

void speedLeftCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print("speedLeftCharacteristic event, written : ");
  motorLeftSpeed = speedLeftCharacteristic.value();
  Serial.println(motorLeftSpeed);

  changeRunState();
}

void speedRightCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  Serial.print("speedRightCharacteristic event, written : ");
  motorRightSpeed = speedRightCharacteristic.value();
  Serial.println(motorRightSpeed);

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
      motorRun(motorLeftSpeed,motorRightSpeed);
      break;
    case MACHINE_RIGHT:
      motorRight(motorLeftSpeed,motorRightSpeed);
      break;
    case MACHINE_LEFT:
      motorLeft(motorLeftSpeed,motorRightSpeed);
      break;
    case MACHINE_BACKWARD:
      motorBack(motorLeftSpeed,motorRightSpeed);
      break;
    default:
      motorStop();
      break;
  }
}
