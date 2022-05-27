#include <Wire.h>
#include <EEPROM.h>
#include "KalmanFilter.h"

TwoWire HWire(2, I2C_FAST_MODE);

int     loop_timer, val;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
long    gyro_x_cal, gyro_y_cal, gyro_z_cal;
float   gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;
int16_t temperature_raw;
float   temperature;

int     loop_status = 0;
int     mode_blinker = 0;
int     debug_mode_delay = 0;

long  acc_total_vector;
float angle_pitch = 0, angle_roll = 0;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;

float pid_error_temp;
float pid_i_mem_pitch, pid_pitch_setpoint = 0, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_roll, pid_roll_setpoint = 0, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;

float pid_p_gain_pitch = EEPROM.read(0) / 100;
float pid_i_gain_pitch = EEPROM.read(1) / 100000;
float pid_d_gain_pitch = EEPROM.read(2);
float pid_p_gain_roll  = EEPROM.read(3) / 100;
float pid_i_gain_roll  = EEPROM.read(4) / 100000;
float pid_d_gain_roll  = EEPROM.read(5);

int   pid_max_roll  =  500; //Maximum output of the PID-controller (+/-).
int   pid_max_pitch =  500; //Maximum output of the PID-controller (+/-).

const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_READ_ADDRESS = 0x3B;
int           esc_1 = 1000, esc_2 = 1000, esc_3 = 1000, esc_4 = 1000;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);
float kalPitch = 0;
float kalRoll = 0;

void setup() {
  // init communcation ways
  pinMode(PC13, OUTPUT);
  
  Serial.begin(115200);
  HWire.setClock(400000);
  HWire.begin();
  delay(250);

  // init drivers
  initIMU();
  initEscDriver();
  calibrateGyro();
  digitalWrite(PC13, HIGH);
  
  //delay(3000);
  //gyro_x_cal= -5371;
  //gyro_y_cal= -258;
  //gyro_z_cal= -102;
       
}

void loop() {

  // PREPARING SYSTEM
  TIMER4_BASE->ARR = 5000;
  serialHandler();
  serialLinkHandler();
  calcAngles();

  // STABILIZE MODE
  if(loop_status == 1){
    calcPid();  
    esc_1 = 1100 + 125 - pid_output_pitch;
    esc_2 = 1100 + 125 + pid_output_pitch;
    
    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;
    if (esc_1 > 1500) esc_1 = 1500;
    if (esc_2 > 1500) esc_2 = 1500; 
    if (esc_3 > 1500) esc_3 = 1500;
    if (esc_4 > 1500) esc_4 = 1500;}
     
  // BOOT MODE
  else {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;}

  // EMITTING ESC SIGNALS
  TIMER4_BASE -> CCR1 = esc_1;
  TIMER4_BASE -> CCR2 = esc_2;
  TIMER4_BASE -> CCR3 = esc_3;
  TIMER4_BASE -> CCR4 = esc_4;
  
  while (loop_timer > micros());
  loop_timer = micros() + 4000;
}
