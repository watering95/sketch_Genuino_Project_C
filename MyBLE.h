BLEService machineService("BBB0");
BLECharacteristic machineStateChara("BBB1", BLERead | BLENotify, 30);

BLEService motorService("0174");
BLEIntCharacteristic operateChara("0175", BLEWrite);
BLECharacteristic speedChara("0176", BLEWrite, 10);

BLEService operateService("0177");
BLEIntCharacteristic modeChara("0178", BLEWrite);
BLECharacteristic pidGainChara("0179", BLEWrite, 20);

char machineState[30];

void initBLE();
void sendBLE();
void blePeripheralConnectHandler(BLEDevice);
void blePeripheralDisconnectHandler(BLEDevice);
void speedCharacteristicWritten(BLEDevice, BLECharacteristic);
void operateCharacteristicWritten(BLEDevice, BLECharacteristic);
void modeCharacteristicWritten(BLEDevice, BLECharacteristic);
void pidGainCharacteristicWritten(BLEDevice, BLECharacteristic);
