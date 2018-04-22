BLEService machineService("BBB0");
BLECharacteristic machineStateChara("BBB1", BLERead | BLENotify, 30);

BLEService motorService("0174");
BLEIntCharacteristic stateChara("0175", BLEWrite);
BLEIntCharacteristic speedLeftChara("0176", BLEWrite);
BLEIntCharacteristic speedRightChara("0177", BLEWrite);
BLEIntCharacteristic isAutoChara("0178", BLEWrite);

char machineState[30];

void initBLE();
void sendBLE();
void blePeripheralConnectHandler(BLEDevice);
void blePeripheralDisconnectHandler(BLEDevice);
void speedLeftCharacteristicWritten(BLEDevice, BLECharacteristic);
void speedRightCharacteristicWritten(BLEDevice, BLECharacteristic);
void stateCharacteristicWritten(BLEDevice, BLECharacteristic);
void isAutoCharacteristicWritten(BLEDevice, BLECharacteristic);
