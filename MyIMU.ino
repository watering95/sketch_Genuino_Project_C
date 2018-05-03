#include "MyIMU.h"

void initIMU() {
  String data; 
  float tmp_roll = 0, tmp_pitch = 0, tmp_yaw = 0;
  int tmp_ax = 0, tmp_ay = 0, tmp_az = 0;
  int tmp_gx = 0, tmp_gy = 0, tmp_gz = 0;
  
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
    tmp_ax += ax;    tmp_ay += ay;    tmp_az += az;
    tmp_gx += gx;    tmp_gy += gy;    tmp_gz += gz;
    delay(50);
  }  
  base_ax = tmp_ax / base;  base_ay = tmp_ay / base;  base_az = tmp_az / base;
  base_gx = tmp_gx / base;  base_gy = tmp_gy / base;  base_gz = tmp_gz / base;
  
  for(int i=0; i < base; i++) {
    readIMU();
    tmp_roll += filtered_angle_roll;
    tmp_pitch += filtered_angle_pitch;
    tmp_yaw += filtered_angle_yaw;
    
    delay(50);
  }  
  base_roll = tmp_roll / base;
  base_yaw = tmp_yaw / base;
  base_pitch = tmp_pitch / base;

  if(az < -10000) base_roll -= 180;
  else base_roll += 90;
} 

void readIMU() {
  const float RADIANS_TO_DEGREES = 180/3.14159;
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131.0;
  const float ALPHA = 0.96; // 입력주기 0.04, 시간상수 1
  
  int accel_x = 0, accel_y = 0, accel_z = 0;
  int gyro_x = 0, gyro_y = 0, gyro_z = 0;

  float accel_yz = 0, accel_xz = 0;

  float accel_angle_roll = 0;
  float accel_angle_pitch = 0;

  float gyro_angle_roll = 0;
  float gyro_angle_pitch = 0;
  float gyro_angle_yaw = 0;
  
  CurieIMU.readGyro(gx, gy, gz);
  CurieIMU.readAccelerometer(ax, ay, az); 

  accel_x = (ax - base_ax);  accel_y = (ay - base_ay);  accel_z = (az - base_az);

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_roll = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_pitch = atan(accel_x / accel_yz) * RADIANS_TO_DEGREES;

  gyro_x = (gx - base_gx) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_y = (gy - base_gy) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_z = (gz - base_gz) / GYROXYZ_TO_DEGREES_PER_SEC - 1;

  gyro_angle_roll = gyro_x * dt / 1000.0;
  gyro_angle_pitch = 0 - gyro_y * dt / 1000.0;
  gyro_angle_yaw = 0 - gyro_z * dt / 1000.0;
  
  filtered_angle_roll = (ALPHA * (filtered_angle_roll + gyro_angle_roll)) + ((1.0 - ALPHA) * accel_angle_roll);
  filtered_angle_pitch = (ALPHA * (filtered_angle_pitch + gyro_angle_pitch)) + ((1.0 - ALPHA) * accel_angle_pitch);
  filtered_angle_yaw += gyro_angle_yaw;

  filtered_angle_roll = angle360(filtered_angle_roll);
  filtered_angle_pitch = angle360(filtered_angle_pitch);
  filtered_angle_yaw = angle360(filtered_angle_yaw);
}

float angle360(float angle) {
  if(angle < 0) angle += 360;
  else if(angle > 360) angle -= 360;
  return angle;
}

void sendToProcessing() {
  Serial.print("Orientation: ");
  Serial.print(angle_yaw);
  Serial.print(" ");
  Serial.print(angle_pitch);
  Serial.print(" ");
  Serial.println(angle_roll);
}

