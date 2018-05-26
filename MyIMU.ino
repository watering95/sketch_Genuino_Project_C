void initIMU() {
  float tmp_yaw = 0;
  int tmp_gz = 0;
  
  Serial.println("Initializing IMU device..."); 
  CurieIMU.begin(); 
 
  CurieIMU.setGyroRate(25); 
  CurieIMU.setGyroRange(250); 

  for(int i=0; i < base; i++) {
    CurieIMU.readGyro(gx, gy, gz); 
    tmp_gz += gz;
    delay(50);
  }  

  base_gz = tmp_gz / base;
  
  for(int i=0; i < base; i++) {
    readIMU();
    tmp_yaw += filtered_angle_yaw;
    
    delay(50);
  }  
  base_yaw = tmp_yaw / base;
} 

void readIMU() {
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131.0;  
  int gyro_z = 0;
  float gyro_angle_yaw = 0;
  
  CurieIMU.readGyro(gx, gy, gz);

  gyro_z = (gz - base_gz) / GYROXYZ_TO_DEGREES_PER_SEC;
  gyro_angle_yaw = 0 - gyro_z * dt_imu / 1000.0;
  filtered_angle_yaw += gyro_angle_yaw;
}

