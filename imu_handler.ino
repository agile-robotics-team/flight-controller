void calcAngles(){
  readIMU();
  
  gyro_x = gyro_x_raw - gyro_x_cal;
  gyro_y = gyro_y_raw - gyro_y_cal;
  gyro_z = gyro_z_raw - gyro_z_cal;

  angle_yaw_output = (angle_yaw_output * 0.7) + (((float)gyro_z / 65.5) * 0.3);      //Gyro pid input is deg/sec.
  
  angle_pitch += gyro_x * 0.0000611;
  angle_roll  += gyro_y * 0.0000611;            
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);
  
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)*  57.296;
  angle_roll_acc  = asin((float)acc_x/acc_total_vector)* -57.296;

  // fix drift
  angle_pitch = angle_pitch * 0.95 + angle_pitch_acc * 0.05;     
  angle_roll  = angle_roll  * 0.95 + angle_roll_acc  * 0.05; 

  angle_pitch_output = kalmanX.update(angle_pitch_acc, angle_pitch);
  if(angle_pitch_output>=90) angle_pitch_output = 90;
  else if (angle_pitch_output<=-90) angle_pitch_output = -90;

  angle_roll_output = kalmanY.update(angle_roll_acc, angle_roll) - 0.70;
  if(angle_roll_output>=90) angle_roll_output = 90;
  else if (angle_roll_output<=-90) angle_roll_output = -90;
}

void calcTemprature() {
  temperature = (float) temperature_raw / 340 + 36.53;
}

void readIMU() {
  HWire.beginTransmission(MPU6050_ADDRESS);
  HWire.write(MPU6050_READ_ADDRESS);
  HWire.endTransmission();
  HWire.requestFrom(MPU6050_ADDRESS, 14);
  acc_x           = HWire.read() << 8 | HWire.read();
  acc_y           = HWire.read() << 8 | HWire.read();
  acc_z           = HWire.read() << 8 | HWire.read();
  gyro_x_raw      = HWire.read() << 8 | HWire.read();
  gyro_y_raw      = HWire.read() << 8 | HWire.read();
  gyro_z_raw      = HWire.read() << 8 | HWire.read();
  temperature_raw = HWire.read() << 8 | HWire.read();
}

void calibrateGyro(){                                              
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  
    if(cal_int % 100 == 0) Serial.println(".");                            
    readIMU();                                             
    gyro_x_cal += gyro_x_raw;                                           
    gyro_y_cal += gyro_y_raw;                                              
    gyro_z_cal += gyro_z_raw;                                              
    delay(3);                                                          
  }
  gyro_x_cal /= 2000;                                                  
  gyro_y_cal /= 2000;                                                  
  gyro_z_cal /= 2000;
  //Serial.print(gyro_x_cal);
  //Serial.print("\t");
  //Serial.print(gyro_y_cal);
  //Serial.print("\t");
  //Serial.print(gyro_z_cal);
  //Serial.print("\t");
  
}

void initIMU() {
  //Awake the MPU6050 sensor from sleep mode
  HWire.beginTransmission(MPU6050_ADDRESS);                             
  HWire.write(0x6B);                                                   
  HWire.write(0x00);                                                   
  HWire.endTransmission();                                      
  
  //Configure the accelerometer (+/-8g)
  HWire.beginTransmission(MPU6050_ADDRESS);                           
  HWire.write(0x1C);                                                    
  HWire.write(0x10);                                                  
  HWire.endTransmission();                                             
  
  //Configure the gyro (500dps full scale)
  HWire.beginTransmission(MPU6050_ADDRESS);                           
  HWire.write(0x1B);                                                   
  HWire.write(0x08);                                              
  HWire.endTransmission();
  
  //Set Digital Low Pass Filter to ~43Hz
  HWire.beginTransmission(MPU6050_ADDRESS);                             
  HWire.write(0x1A);                                                    
  HWire.write(0x03);                                                    
  HWire.endTransmission();
}
