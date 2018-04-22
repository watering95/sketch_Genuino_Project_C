#include "MyIMU.h"

void initIMU() {
  String data; 
  int tmpGx = 0, tmpGy = 0, tmpGz = 0, tmpAx = 0, tmpAy = 0, tmpAz = 0;
  
  Serial.println("Initializing IMU device..."); 
  CurieIMU.begin(); 
 
  CurieIMU.setGyroRate(25); 
  CurieIMU.setGyroRange(250); 

  CurieIMU.setAccelerometerRate(25); 
  CurieIMU.setAccelerometerRange(2); 
  
  #ifdef MADGWICK
  filter.begin(25);
  #endif
  
  for(int i=0; i < base; i++) {
    CurieIMU.readGyro(gx, gy, gz);
    CurieIMU.readAccelerometer(ax, ay, az);
    tmpGx += gx;    tmpGy += gy;    tmpGz += gz;
    tmpAx += ax;    tmpAy += ay;    tmpAz += az;
    delay(50);
/*
    data = String(String(tmpGx) + "," + String(tmpGy) + "," + String(tmpGz) + "," + String(tmpAx) + "," + String(tmpAy) + "," + String(tmpAz) + ",");
    Serial.println(data);
*/
  }  

/*// IMU 초기값 
  Serial.println("average");
  baseGx = tmpGx/base;  baseGy = tmpGy/base;  baseGz = tmpGz/base;
  baseAx = tmpAx/base;  baseAy = tmpAy/base;  baseAz = tmpAz/base;
  data = String(String(baseGx) + "," + String(baseGy) + "," + String(baseGz) + "," + String(baseAx) + "," + String(baseAy) + "," + String(baseAz) + ",");
  Serial.println(data);
*/
} 

void readIMU() {
#ifdef MADGWICK // Madgwick 사용 
  readIMUWithMadgwick();

#else // Madgwick 미사용
  const float RADIANS_TO_DEGREES = 180/3.14159;
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131.0;
  const float ALPHA = 0.96; // 입력주기 0.04, 시간상수 1
  
  int accel_x = 0, accel_y = 0, accel_z = 0;
  int gyro_x = 0, gyro_y = 0, gyro_z = 0;

  float accel_yz = 0, accel_xz = 0;

  float accel_angle_roll = 0;
  float accel_angle_pitch = 0;
  float accel_angle_yaw = 0;

  float gyro_angle_roll = 0;
  float gyro_angle_pitch = 0;
  float gyro_angle_yaw = 0;
  
  CurieIMU.readGyro(gx, gy, gz);
  CurieIMU.readAccelerometer(ax, ay, az); 

  accel_x = (ax - baseAx);  accel_y = (ay - baseAy);  accel_z = (az - baseAz) + 16384;

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_roll = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_pitch = atan(accel_x / accel_yz) * RADIANS_TO_DEGREES;

  gyro_x = (gx - baseGx) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (gy - baseGy) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (gz - baseGz) / GYROXYZ_TO_DEGREES_PER_SEC - 1;

  gyro_angle_roll = gyro_x * dt / 1000.0;
  gyro_angle_pitch = 0 - gyro_y * dt / 1000.0;
  gyro_angle_yaw = 0 - gyro_z * dt / 1000.0;
  
  filtered_angle_roll = (ALPHA * (filtered_angle_roll + gyro_angle_roll)) + ((1.0 - ALPHA) * accel_angle_roll);
  filtered_angle_pitch = (ALPHA * (filtered_angle_pitch + gyro_angle_pitch)) + ((1.0 - ALPHA) * accel_angle_pitch);
  filtered_angle_yaw += gyro_angle_yaw;

  filtered_angle_roll = angle360(filtered_angle_roll);
  angle_roll = filtered_angle_roll + 1.68 + 25 - 77;
  filtered_angle_pitch = angle360(filtered_angle_pitch);
  angle_pitch = filtered_angle_pitch + 4.71;
  filtered_angle_yaw = angle360(filtered_angle_yaw);
  angle_yaw = filtered_angle_yaw;

  // data check
/*
  String strAngleX = String(gyro_angle_roll, 3);
  String strAngleY = String(gyro_angle_pitch, 3);
  String strAngleZ = String(gyro_angle_yaw, 3);
*/

#endif
/*  
  String strAngleX = String(0);
  String strAngleY = String(0);
  String strAngleZ = String(gyro_z);
  String strAngleX = String(filtered_angle_roll, 3);
  String strAngleY = String(filtered_angle_pitch, 3);
  String strAngleZ = String(filtered_angle_yaw, 3);
  String sendData = String(strAngleX + "," + strAngleY + "," + strAngleZ + ",");
  Serial.println(sendData);
*/
}

#ifdef MADGWICK // Madgwick 사용 
void readIMUWithMadgwick() {
  float accel_x = 0, accel_y = 0, accel_z = 0;
  float gyro_x = 0, gyro_y = 0, gyro_z = 0;
  
  CurieIMU.readGyro(gx, gy, gz);
  CurieIMU.readAccelerometer(ax, ay, az); 

  accel_x = convertRawAcceleration(ax);
  accel_y = convertRawAcceleration(ay);
  accel_z = convertRawAcceleration(az);

  gyro_x = convertRawGyro(gx);
  gyro_y = convertRawGyro(gy);
  gyro_z = convertRawGyro(gz);

  filter.updateIMU(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);

  filtered_angle_roll = filter.getRoll();
  filtered_angle_pitch = filter.getPitch();
  filtered_angle_yaw = filter.getYaw();
}
#endif

float convertRawAcceleration(int raw) {
  float a = (raw * 2.0) / 32768.0;
  return a;  
}

float convertRawGyro(int raw) {
  float g = (raw * 250.0) / 32768.0;
  return g;
}

float angle360(float angle) {
  if(angle < -360) angle += 360;
  else if(angle > 360) angle -= 360;
  return angle;
}
