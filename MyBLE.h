BLEService machineService("BBB0");
BLECharacteristic machineStateChara("BBB1", BLERead | BLENotify, 30);

BLEService motorService("0174");
BLEIntCharacteristic operateChara("0175", BLEWrite);
BLEIntCharacteristic speedLeftChara("0176", BLEWrite);
BLEIntCharacteristic speedRightChara("0177", BLEWrite);
BLEIntCharacteristic modeChara("0178", BLEWrite);

char machineState[30];

void initBLE();
void sendBLE();
void blePeripheralConnectHandler(BLEDevice);
void blePeripheralDisconnectHandler(BLEDevice);
void speedLeftCharacteristicWritten(BLEDevice, BLECharacteristic);
void speedRightCharacteristicWritten(BLEDevice, BLECharacteristic);
void operateCharacteristicWritten(BLEDevice, BLECharacteristic);
void modeCharacteristicWritten(BLEDevice, BLECharacteristic);
