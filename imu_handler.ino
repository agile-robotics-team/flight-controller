void calcAngles(){
  readIMU();

  // pid calculations
  gyro_pitch  = gyro_pitch_raw  - gyro_pitch_cal;
  gyro_roll   = gyro_roll_raw   - gyro_roll_cal;
  gyro_yaw    = gyro_yaw_raw    - gyro_yaw_cal;
  
  gyro_roll_input  = (gyro_roll_input   * 0.7) + (((float)gyro_roll   / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input  * 0.7) + (((float)gyro_pitch  / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_yaw_input   = (gyro_yaw_input    * 0.7) + (((float)gyro_yaw    / 65.5) * 0.3);       //Gyro pid input is deg/sec.

  // angle calculations
  angle_pitch += gyro_pitch * 0.0000611;
  angle_roll  += gyro_roll  * 0.0000611;            
  angle_pitch += angle_roll  * sin(gyro_yaw * 0.000001066);
  angle_roll  -= angle_pitch * sin(gyro_yaw * 0.000001066);
  acc_total_vector = sqrt((acc_pitch*acc_pitch)+(acc_roll*acc_roll)+(acc_yaw*acc_yaw));
  if (abs(acc_pitch) < acc_total_vector) angle_pitch_acc = asin((float)acc_pitch / acc_total_vector) * 57.296;
  if (abs(acc_roll)  < acc_total_vector) angle_roll_acc  = asin((float)acc_roll  / acc_total_vector) * 57.296; 
  angle_pitch = (angle_pitch * 0.96 + angle_pitch_acc * 0.04);     
  angle_roll  = (angle_roll  * 0.96 + angle_roll_acc  * 0.04);
  angle_pitch_output  = kalmanPitch.update(angle_pitch_acc + 0.2, angle_pitch + 0.2);
  angle_roll_output   = kalmanRoll.update (angle_roll_acc + 1.7,  angle_roll + 1.7);
}

void readIMU() {
  HWire.beginTransmission(MPU6050_ADDRESS);
  HWire.write(MPU6050_READ_ADDRESS);
  HWire.endTransmission();
  HWire.requestFrom(MPU6050_ADDRESS, 14);
  acc_roll         = HWire.read() << 8 | HWire.read();
  acc_pitch          = HWire.read() << 8 | HWire.read();
  acc_yaw           = HWire.read() << 8 | HWire.read();
  temperature_raw   = HWire.read() << 8 | HWire.read();
  gyro_pitch_raw    = HWire.read() << 8 | HWire.read();
  gyro_roll_raw     = HWire.read() << 8 | HWire.read();
  gyro_yaw_raw      = HWire.read() << 8 | HWire.read();
  gyro_roll_raw *= -1;
  temperature = (float) temperature_raw / 340 + 36.53;
}

void calibrateGyro(){                                              
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  
    if(cal_int % 100 == 0) Serial.println(".");                            
    readIMU();                                             
    gyro_pitch_cal  += gyro_pitch_raw;                                           
    gyro_roll_cal   += gyro_roll_raw;                                              
    gyro_yaw_cal    += gyro_yaw_raw;                                              
    delay(4);                                                          
  }
  gyro_pitch_cal /= 1000;                                                  
  gyro_roll_cal /= 1000;                                                  
  gyro_yaw_cal /= 1000;
  Serial.print(gyro_pitch_cal);
  Serial.print("\t");
  Serial.print(gyro_roll_cal);
  Serial.print("\t");
  Serial.println(gyro_yaw_cal);

  /*gyro_pitch_cal = -280;
  gyro_roll_cal = 102;
  gyro_yaw_cal = 97;*/
     

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
