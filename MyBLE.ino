#include "MyBLE.h"

void initBLE() {
  BLE.begin();
  
  BLE.setLocalName("GENUINO101");

  BLE.setAdvertisedService(machineService);
  machineService.addCharacteristic(machineStateChara);
  
  BLE.setAdvertisedService(motorService);
  motorService.addCharacteristic(operateChara);
  motorService.addCharacteristic(speedLeftChara);
  motorService.addCharacteristic(speedRightChara);
  motorService.addCharacteristic(modeChara);

  BLE.addService(machineService);
  BLE.addService(motorService);
  
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  operateChara.setEventHandler(BLEWritten, operateCharacteristicWritten);
  speedLeftChara.setEventHandler(BLEWritten, speedLeftCharacteristicWritten);
  speedRightChara.setEventHandler(BLEWritten, speedRightCharacteristicWritten);
  modeChara.setEventHandler(BLEWritten, modeCharacteristicWritten);
  
  BLE.advertise();
  Serial.println("BLE Genuino101 Peripheral");
}

void sendBLE() {
  if(!isConnectedCentral) return;
  
  String strState = String(state);
  String strAngleX = String(filtered_angle_roll, 1);
  String strAngleY = String(filtered_angle_pitch, 1);
  String strAngleZ = String(filtered_angle_yaw, 1);
  String sendData = String(strState + "," + strAngleX + "," + strAngleY + "," + strAngleZ + ",");
  sendData.toCharArray(machineState,sendData.length()+1);

  Serial.println(machineState);
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

  if(setLeftSpeed < 150) setLeftSpeed = 150;
  if(now_vr < 150) now_vr = 150;

  changeSpeed(10, setLeftSpeed, now_vr);
}

void speedRightCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("speedRightCharacteristic event, written : ");
  setRightSpeed = speedRightChara.value();
  Serial.println(setRightSpeed);

  if(setRightSpeed < 150) setRightSpeed = 150;
  if(now_vl < 150) now_vl = 150;

  changeSpeed(10, now_vl, setRightSpeed);
}

void operateCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("operateCharacteristic event, written : ");
  operate = operateChara.value();
  Serial.println(operate);

  if(mode == MODE_MANUAL) {
    manualOperate();
    changeSpeed(10, setLeftSpeed, setRightSpeed);
  }
}

void modeCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  Serial.print("modeCharacteristic event, written : ");
  if(modeChara.value() == MODE_AUTO) mode = MODE_AUTO;
  else {
    mode = MODE_MANUAL;
    Stop();
  }
  initIMU();
  Serial.println(mode);
}
