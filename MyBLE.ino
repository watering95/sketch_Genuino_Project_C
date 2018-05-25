#include "MyBLE.h"

void initBLE() {
  BLE.begin();
  
  BLE.setLocalName("GENUINO101");

  BLE.setAdvertisedService(machineService);
  machineService.addCharacteristic(machineStateChara);
  
  BLE.setAdvertisedService(motorService);
  motorService.addCharacteristic(operateChara);
  motorService.addCharacteristic(speedChara);

  BLE.setAdvertisedService(operateService);
  operateService.addCharacteristic(modeChara);
  operateService.addCharacteristic(pidGainChara);

  BLE.addService(machineService);
  BLE.addService(motorService);
  BLE.addService(operateService);
  
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  operateChara.setEventHandler(BLEWritten, operateCharacteristicWritten);
  speedChara.setEventHandler(BLEWritten, speedCharacteristicWritten);
  modeChara.setEventHandler(BLEWritten, modeCharacteristicWritten);
  pidGainChara.setEventHandler(BLEWritten, pidGainCharacteristicWritten);
  
  BLE.advertise();
  Serial.println("BLE Genuino101 Peripheral");
}

void sendBLE() {
  if(!isConnectedCentral) return;
  
  String strMode = String(mode);
  String strState = String(state);
  String strAngleTarget = String(targetAngle, 1);
  String strAngleYaw = String(filtered_angle_yaw, 1);
  String strOutput = String(output, 1);
  String sendData = String(strMode + "," + strState + "," + strAngleTarget + "," + strAngleYaw + ","+ strOutput + ",");
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

void speedCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  const byte* byteBuffer = (const byte* )malloc(10);
  byteBuffer = speedChara.value();
  
  String strBuffer = String((char *)byteBuffer);

  Serial.println(strBuffer);
  
  setLeftSpeed = strBuffer.substring(0,3).toInt();
  setRightSpeed = strBuffer.substring(5,7).toInt();

  setLeftSpeed = minimumSpeed + (maxSpeed - minimumSpeed) * (setLeftSpeed / 100.0);
  setRightSpeed = minimumSpeed + (maxSpeed - minimumSpeed) * (setRightSpeed / 100.0);

  Serial.print("speedCharacteristic event, written : ");
  Serial.print(setLeftSpeed);
  Serial.print(",");
  Serial.println(setRightSpeed);
  
  changeSpeed(10, setLeftSpeed, setRightSpeed);
}

void operateCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  new_operate = operateChara.value();
  
  Serial.print("operateCharacteristic event, written : ");
  Serial.println(new_operate);

  if(mode == MODE_MANUAL) {
    manualOperate();
    changeSpeed(10, setLeftSpeed, setRightSpeed);
  }
}

void modeCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  if(modeChara.value() == MODE_AUTO) mode = MODE_AUTO;
  else {
    mode = MODE_MANUAL;
    Stop();
  }
  initIMU();
  
  Serial.print("modeCharacteristic event, written : ");
  Serial.println(mode);
}

void pidGainCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  const byte* byteBuffer = (const byte* )malloc(20);
  byteBuffer = pidGainChara.value();
  
  String strBuffer = String((char *)byteBuffer);
  kp = strBuffer.substring(0,3).toInt();
  ki = strBuffer.substring(5,7).toInt();
  kd = strBuffer.substring(9,11).toInt();
  
  Serial.print("pidGainCharacteristic event, written : ");
  Serial.print(kp);
  Serial.print(",");
  Serial.print(ki);
  Serial.print(",");
  Serial.print(kd);
}
