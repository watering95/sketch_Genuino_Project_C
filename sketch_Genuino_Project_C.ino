#include <CurieBLE.h>
#include "CurieIMU.h"

#define AVERAGE 5
#define MOVERANGE 0.1
#define AXIS_X  0
#define AXIS_Y  1
#define AXIS_Z  2

#define PIN_DATA 8
#define PIN_CLOCK 4
#define PIN_LATCH 12

#define MOTOR_RIGHT 11  //H-Bridge M1
#define MOTOR_LEFT 3    //H-Bridge M2

#define SPEED 255

BLEPeripheral blePeripheral;
BLEService genuinoService("BBB0");
/*
BLEFloatCharacteristic gyroXChar("BBB1", BLERead | BLENotify);
BLEFloatCharacteristic gyroYChar("BBB2", BLERead | BLENotify);
BLEFloatCharacteristic gyroZChar("BBB3", BLERead | BLENotify);
BLEFloatCharacteristic acclXChar("BBB4", BLERead | BLENotify);
BLEFloatCharacteristic acclYChar("BBB5", BLERead | BLENotify);
BLEFloatCharacteristic acclZChar("BBB6", BLERead | BLENotify);
*/
BLEFloatCharacteristic posXChar("BBB7", BLERead | BLENotify);
BLEFloatCharacteristic posYChar("BBB8", BLERead | BLENotify);
BLEFloatCharacteristic posZChar("BBB9", BLERead | BLENotify);
BLEFloatCharacteristic speedXChar("BBBA", BLERead | BLENotify);
BLEFloatCharacteristic speedYChar("BBBB", BLERead | BLENotify);
BLEFloatCharacteristic speedZChar("BBBC", BLERead | BLENotify);
/*
BLEDescriptor gyroXDescriptor = BLEDescriptor("2901", "gyroX");
BLEDescriptor gyroYDescriptor = BLEDescriptor("2901", "gyroY");
BLEDescriptor gyroZDescriptor = BLEDescriptor("2901", "gyroZ");
BLEDescriptor acclXDescriptor = BLEDescriptor("2901", "acclX");
BLEDescriptor acclYDescriptor = BLEDescriptor("2901", "acclY");
BLEDescriptor acclZDescriptor = BLEDescriptor("2901", "acclZ");
*/
BLEDescriptor posXDescriptor = BLEDescriptor("2901", "posX");
BLEDescriptor posYDescriptor = BLEDescriptor("2901", "posY");
BLEDescriptor posZDescriptor = BLEDescriptor("2901", "posZ");
BLEDescriptor speedXDescriptor = BLEDescriptor("2901", "speedX");
BLEDescriptor speedYDescriptor = BLEDescriptor("2901", "speedY");
BLEDescriptor speedZDescriptor = BLEDescriptor("2901", "speedZ");

struct Matrix_2_2 {
  int row = 2;
  int col = 2;
  float mat[2][2] = {{0,}, {0,}};
};

struct Matrix_2_1 {
  int row = 2;
  int col = 1;
  float mat[2][1] = {{0}, {0}};
};

Matrix_2_2 H, I, R[3], Q[3], A[3], P_[3], P[3], K[3];
Matrix_2_1 X_hat_[3], X_hat[3], Z[3], B[3];
unsigned long movetime[2] = {0, 0};
float init_a[3], g[3][AVERAGE], a[3][AVERAGE];
float avg_g[3][2], avg_a[3][2];
double pos[3][2], velocity[3][2];
boolean isInitialized = false;

void setup() {
  // put your setup code here, to run once:
  H.mat[0][0] = 1;      H.mat[1][1] = 1;
  I.mat[0][0] = 1;      I.mat[1][1] = 1;

  for (int axis = AXIS_X; axis <= AXIS_Z; axis++) {
    for (int i = 0; i < AVERAGE; i++) {
      g[axis][i] = 0;  a[axis][i] = 0;
    }
    velocity[axis][0] = velocity[axis][1] = 0;
    pos[axis][0] = pos[axis][1] = 0;
    Q[axis].mat[0][0] = 1;      Q[axis].mat[1][1] = 10;
    R[axis].mat[0][0] = 1;      R[axis].mat[1][1] = 10;
  }

  Serial.begin(9600);
  initIMU();
  initBLE();
  initMotorShield();
}

void loop() {
  static int index = 0;
  static boolean isPrint = false;
  unsigned long millisec = 0;
  float sec;

  // put your main code here, to run repeatedly:
  BLECentral central = blePeripheral.central();

  if (central && !isPrint) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    isPrint = true;
    delay(100);
  } else {
    central = blePeripheral.central();
  }

  CurieIMU.readGyroScaled(g[AXIS_X][index], g[AXIS_Y][index], g[AXIS_Z][index]);
  CurieIMU.readAccelerometerScaled(a[AXIS_X][index], a[AXIS_Y][index], a[AXIS_Z][index]);

  movetime[1] = millis();
  millisec = movetime[1] - movetime[0];
  sec = millisec / 1000.0;

  CalculatePosition(index, sec);

  if (central.connected()) {
    sendBLE(index);
  } else isPrint = false;

  if (index < AVERAGE - 1) index++;
  else index = 0;

  movetime[0] = movetime[1];
}

void CalculatePosition(int index, float sec) {
  float range = 0;

  for (int axis = AXIS_X; axis <= AXIS_Z; axis++) {
    switch (axis) {
      case AXIS_X:
      case AXIS_Y:
        range = MOVERANGE;
        break;
      case AXIS_Z:
        range = MOVERANGE + 1;
        break;
    }
    if (!isInitialized) {
      avg_a[axis][0] = filter(a[axis], index + 1);
      if (index == AVERAGE - 1) {
        init_a[axis] = avg_a[axis][0];
        if (axis == AXIS_Z) isInitialized = true;
      }
    } else {
      avg_a[axis][1] = filter(a[axis], AVERAGE);
      if (avg_a[axis][1] < range && avg_a[axis][1] > -range) {
        velocity[axis][0] = velocity[axis][1] = 0;
      } else {
        velocity[axis][1] = Integration(velocity[axis][0], 9.8 * (avg_a[axis][0] - init_a[axis]), 9.8 * (avg_a[axis][1] - init_a[axis]), sec);
        pos[axis][1] = Integration(pos[axis][0], velocity[axis][0], velocity[axis][1], sec);

        //        KalmanFilter(axis, sec);
      }
      avg_a[axis][0] = avg_a[axis][1];
      pos[axis][0] = pos[axis][1];
      velocity[axis][0] = velocity[axis][1];
    }
  }
}

void KalmanFilter(int axis, float sec) {
  A[axis].mat[0][0] = 1;  A[axis].mat[0][1] = sec;  A[axis].mat[1][1] = 1;
  B[axis].mat[0][0] = sec * sec / 2 * avg_a[axis][1];  B[axis].mat[1][0] = sec * avg_a[axis][1];

  Z[axis].mat[0][0] = pos[axis][1];
  Z[axis].mat[1][0] = velocity[axis][1];

  X_hat_[axis] = AddMatrix(ProductMatrix(A[axis], X_hat[axis]), B[axis]);
  P_[axis] = AddMatrix(ProductMatrix(ProductMatrix(A[axis], P[axis]), TransposeMatrix(A[axis])), Q[axis]);

  K[axis] = ProductMatrix(P_[axis], InverseMatrix(AddMatrix(P_[axis], R[axis])));
  X_hat[axis] = AddMatrix(X_hat_[axis], ProductMatrix(K[axis], SubMatrix(Z[axis], ProductMatrix(H, X_hat_[axis]))));
  P[axis] = ProductMatrix(SubMatrix(I, ProductMatrix(K[axis], H)), P_[axis]);

  pos[axis][1] = X_hat[axis].mat[0][0];
  velocity[axis][1] = X_hat[axis].mat[1][0];
}

void initMotorShield() {
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLOCK, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);

  pinMode(MOTOR_RIGHT, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
}

void motorRun(int v) {
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

void motorBack(int v) {
  byte dir = 0x0A;  //  0 0 0 0 1 0 1 0
  changeDirection(dir);
  analogWrite(MOTOR_RIGHT, v);
  analogWrite(MOTOR_LEFT, v);
}

void motorLeft(int v) {
  byte dir = 0x04;  //  0 0 0 0 0 1 0 0
  changeDirection(dir);
  analogWrite(MOTOR_LEFT, v);
  analogWrite(MOTOR_RIGHT, 0);
}

void motorRight(int v) {
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
/*
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
*/
  blePeripheral.addAttribute(posXChar);
  blePeripheral.addAttribute(posXDescriptor);
  blePeripheral.addAttribute(posYChar);
  blePeripheral.addAttribute(posYDescriptor);
  blePeripheral.addAttribute(posZChar);
  blePeripheral.addAttribute(posZDescriptor);
  blePeripheral.addAttribute(speedXChar);
  blePeripheral.addAttribute(speedXDescriptor);
  blePeripheral.addAttribute(speedYChar);
  blePeripheral.addAttribute(speedYDescriptor);
  blePeripheral.addAttribute(speedZChar);
  blePeripheral.addAttribute(speedZDescriptor);

/*
  gyroXChar.setValue(0);  gyroYChar.setValue(0);  gyroZChar.setValue(0);
  acclXChar.setValue(0);  acclYChar.setValue(0);  acclZChar.setValue(0);
*/
  posXChar.setValue(0);   posYChar.setValue(0);   posZChar.setValue(0);
  speedXChar.setValue(0); speedYChar.setValue(0); speedZChar.setValue(0);

  blePeripheral.begin();
  Serial.println("BLE Genuino101 Peripheral");
}

void sendBLE(int index) {
/*
  gyroXChar.setValue(g[AXIS_X][index]);
  gyroYChar.setValue(g[AXIS_Y][index]);
  gyroZChar.setValue(g[AXIS_Z][index]);
  acclXChar.setValue(a[AXIS_X][index]);
  acclYChar.setValue(a[AXIS_Y][index]);
  acclZChar.setValue(a[AXIS_Z][index]);
*/
  posXChar.setValue(pos[AXIS_X][0]);
  posYChar.setValue(pos[AXIS_Y][0]);
  posZChar.setValue(pos[AXIS_Z][0]);
  speedXChar.setValue(velocity[AXIS_X][0]);
  speedYChar.setValue(velocity[AXIS_Y][0]);
  speedZChar.setValue(velocity[AXIS_Z][0]);
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

Matrix_2_2 AddMatrix(Matrix_2_2 A, Matrix_2_2 B) {
  Matrix_2_2 result;

  for (int i = 0; i < A.row; i++) {
    for (int j = 0; j < B.col; j++) {
      result.mat[i][j] = A.mat[i][j] + B.mat[i][j];
    }
  }
  return result;
}

Matrix_2_1 AddMatrix(Matrix_2_1 A, Matrix_2_1 B) {
  Matrix_2_1 result;

  for (int i = 0; i < A.row; i++) {
    for (int j = 0; j < B.col; j++) {
      result.mat[i][j] = A.mat[i][j] + B.mat[i][j];
    }
  }
  return result;
}

Matrix_2_2 SubMatrix(Matrix_2_2 A, Matrix_2_2 B) {
  Matrix_2_2 result;

  for (int i = 0; i < A.row; i++) {
    for (int j = 0; j < B.col; j++) {
      result.mat[i][j] = A.mat[i][j] - B.mat[i][j];
    }
  }
  return result;
}

Matrix_2_1 SubMatrix(Matrix_2_1 A, Matrix_2_1 B) {
  Matrix_2_1 result;

  for (int i = 0; i < A.row; i++) {
    for (int j = 0; j < B.col; j++) {
      result.mat[i][j] = A.mat[i][j] - B.mat[i][j];
    }
  }
  return result;
}

Matrix_2_2 ProductMatrix(Matrix_2_2 A, Matrix_2_2 B) {
  Matrix_2_2 result;

  result.mat[0][0] = A.mat[0][0] * B.mat[0][0] + A.mat[0][1] * B.mat[1][0];
  result.mat[0][1] = A.mat[0][0] * B.mat[0][1] + A.mat[0][1] * B.mat[1][1];
  result.mat[1][0] = A.mat[1][0] * B.mat[0][0] + A.mat[1][1] * B.mat[1][0];
  result.mat[1][1] = A.mat[1][0] * B.mat[0][1] + A.mat[1][1] * B.mat[1][1];

  return result;
}

Matrix_2_1 ProductMatrix(Matrix_2_2 A, Matrix_2_1 B) {
  Matrix_2_1 result;

  result.mat[0][0] = A.mat[0][0] * B.mat[0][0] + A.mat[0][1] * B.mat[1][0];
  result.mat[1][0] = A.mat[1][0] * B.mat[0][0] + A.mat[1][1] * B.mat[1][0];

  return result;
}

Matrix_2_2 InverseMatrix(Matrix_2_2 A) {
  Matrix_2_2 result;

  float det = A.mat[0][0] * A.mat[1][1] - A.mat[0][1] * A.mat[1][0];

  result.mat[0][0] = A.mat[1][1] / det;
  result.mat[0][1] = A.mat[0][1] / det * (-1);
  result.mat[1][0] = A.mat[1][0] / det * (-1);
  result.mat[1][1] = A.mat[0][0] / det;

  return result;
}

Matrix_2_2 TransposeMatrix(Matrix_2_2 A) {
  Matrix_2_2 result;

  result.mat[0][1] = A.mat[1][0];
  result.mat[1][0] = A.mat[0][1];

  return result;
}

