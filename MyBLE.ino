#include "MyBLE.h"

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
