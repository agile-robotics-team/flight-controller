#include <Wire.h>
#include <EEPROM.h>
#include "KalmanFilter.h"

int     loop_timer, val;
int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
long    gyro_x_cal, gyro_y_cal, gyro_z_cal;
float   gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;
int16_t temperature_raw;
float   temperature;

int     telemetry_status = 0;
int     telemetry_delay = 0;
int     mode_status = 3;
int     mode_blinker = 0;

long  acc_total_vector;
float angle_pitch = 0, angle_roll = 0;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;

volatile float pid_error_temp;
float pid_i_mem_pitch, pid_pitch_setpoint = 0, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_roll,  pid_roll_setpoint = 0, pid_output_roll,  pid_last_roll_d_error;
volatile double pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch;
volatile double pid_p_gain_roll, pid_i_gain_roll,pid_d_gain_roll;
int   pid_max_roll  =  400; //Maximum output of the PID-controller (+/-).
int   pid_max_pitch =  400; //Maximum output of the PID-controller (+/-).

float pid_i_mem_yaw, pid_yaw_setpoint = 0, pid_output_yaw, pid_last_yaw_d_error;
float pid_p_gain_yaw = 0.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.0;                //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;

const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_READ_ADDRESS = 0x3B;
int           esc_1 = 1000, esc_2 = 1000, esc_3 = 1000, esc_4 = 1000;
int           throttle;
uint8_t       startup_flag = 0;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

TwoWire HWire(2, I2C_FAST_MODE);

// receiver control
volatile int channels[18];

void setup() {
  
  // init communcation ways
  pinMode(PC13, OUTPUT);
  Serial2.begin(100000, SERIAL_8E2);
  Serial.begin(115200);
  while(!Serial);
  HWire.setClock(400000);
  HWire.begin();
  delay(50);

  // init drivers
  initIMU();
  initEscDriver();
  calibrateGyro();
  digitalWrite(PC13, HIGH);
  
  //delay(3000);
  //gyro_x_cal= -5371;
  //gyro_y_cal= -258;
  //gyro_z_cal= -102;
  
  /*pid_p_gain_pitch = EEPROM.read(0) / 100.0;
  pid_i_gain_pitch = EEPROM.read(1) / 100000.0;
  pid_d_gain_pitch = EEPROM.read(2);
  pid_p_gain_roll  = EEPROM.read(3) / 100.0;
  pid_i_gain_roll  = EEPROM.read(4) / 100000.0;
  pid_d_gain_roll  = EEPROM.read(5);*/
  
  pid_p_gain_pitch = 1.5;
  pid_i_gain_pitch = 0.0;
  pid_d_gain_pitch = 47.0;
  pid_p_gain_roll  = pid_p_gain_pitch;
  pid_i_gain_roll  = pid_i_gain_pitch;
  pid_d_gain_roll  = pid_d_gain_pitch;
}

void loop() {

  // PREPARING SYSTEM FOR CURRENT LOOP
  calcAngles();
  sbusProcess();
  calcPid();
  
  // BOOT MODE
  if(mode_status == 0){
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
    startup_flag = 0;}

  // DEBUG MODE
  else if(mode_status == 1) startup_flag = 0;
  
  // STABILIZE MODE
  else if(mode_status == 2){
      if(startup_flag == 0){
      angle_pitch = angle_pitch_acc;   
      angle_roll = angle_roll_acc; 
      pid_i_mem_roll = 0;
      pid_last_roll_d_error = 0;
      pid_i_mem_pitch = 0;
      pid_last_pitch_d_error = 0;
      pid_i_mem_yaw = 0;
      pid_last_yaw_d_error = 0;
      startup_flag = 1;
      }
    
    esc_1 =  1100 + channels[0] - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 =  1100 + channels[0] + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 =  1100 + channels[0] + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 =  1100 + channels[0] - pid_output_pitch - pid_output_roll + pid_output_yaw; 
    
    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;
    if (esc_1 > 1500) esc_1 = 1500;
    if (esc_2 > 1500) esc_2 = 1500; 
    if (esc_3 > 1500) esc_3 = 1500;
    if (esc_4 > 1500) esc_4 = 1500;
    }
  
  // EMITTING ESC SIGNALS
  TIMER4_BASE->CCR1 = esc_1;
  TIMER4_BASE->CCR2 = esc_2;
  TIMER4_BASE->CCR3 = esc_3;
  TIMER4_BASE->CCR4 = esc_4; 
  TIMER4_BASE->CNT = 5000; 

  // Serial.println(loop_timer - micros());
  serialHandler();
  telemetryHandler();
  while (loop_timer > micros());
  loop_timer = micros() + 4000;
}
