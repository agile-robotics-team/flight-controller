#include <Wire.h>
#include <EEPROM.h>
#include "KalmanFilter.h"

int32_t loop_timer, cache_timer;
int     val = 0;
int16_t gyro_pitch_raw, gyro_roll_raw, gyro_yaw_raw;
long    gyro_pitch_cal, gyro_roll_cal, gyro_yaw_cal;
float   gyro_pitch, gyro_roll, gyro_yaw;
int16_t acc_pitch,  acc_roll,  acc_yaw;
int16_t temperature_raw;
float   temperature;

int           telemetry_status = 0;
int           telemetry_delay = 0;
volatile int  mode_status = 0;
int           mode_blinker = 0;

long  acc_total_vector;
float angle_pitch = 0, angle_roll = 0;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;
float angle_pitch_output_kalman, angle_roll_output_kalman;
float gyro_roll_input, gyro_pitch_input, gyro_yaw_input;

float pid_error_temp;
float roll_level_adjust, pitch_level_adjust;
float pid_i_mem_pitch, pid_pitch_setpoint = 0, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_roll,  pid_roll_setpoint = 0, pid_output_roll,  pid_last_roll_d_error;

float pid_p_gain_pitch = 0.47;
float pid_i_gain_pitch = 0.0; // 01;
float pid_d_gain_pitch = 9.3;
float pid_p_gain_roll  = pid_p_gain_pitch;
float pid_i_gain_roll  = pid_i_gain_pitch;
float pid_d_gain_roll  = pid_d_gain_pitch;
int   pid_max_roll  =  400; //Maximum output of the PID-controller (+/-).
int   pid_max_pitch =  400; //Maximum output of the PID-controller (+/-).

float pid_i_mem_yaw, pid_yaw_setpoint = 0, pid_output_yaw, pid_last_yaw_d_error;
float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller (default = 4.0).
float pid_i_gain_yaw = 0.0;                //Gain setting for the pitch I-controller (default = 0.02).
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller (default = 0.0).
int pid_max_yaw = 400;

const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_READ_ADDRESS = 0x3B;
int           esc_1 = 1000, esc_2 = 1000, esc_3 = 1000, esc_4 = 1000;
int           throttle;
uint8_t       startup_flag = 0;

KalmanFilter kalmanPitch(0.001, 0.003, 0.03);
KalmanFilter kalmanRoll(0.001, 0.003, 0.03);

TwoWire HWire(2, I2C_FAST_MODE);

// receiver control
int channels[18];

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
  loop_timer = micros() + 4000;
  digitalWrite(PC13, HIGH);
}

void loop() {

  // PREPARING SYSTEM FOR CURRENT LOOP
  calcAngles();
  sbusProcess();
  
  // BOOT & DEBUG MODE
  if(mode_status == 0 || mode_status == 1){
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
    startup_flag = 0;}
  
  // STABILIZE MODE
  else if(mode_status == 2){
      if(startup_flag == 0){
      angle_pitch = angle_pitch_acc;   
      angle_roll  = angle_roll_acc; 
      pid_i_mem_roll = 0;
      pid_i_mem_pitch = 0;
      pid_i_mem_yaw = 0;
      pid_last_roll_d_error = 0;
      pid_last_pitch_d_error = 0;
      pid_last_yaw_d_error = 0;
      startup_flag = 1;
      }
      
    calcPid();

    esc_1 =  1100 + channels[0] - pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_2 =  1100 + channels[0] + pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_3 =  1100 + channels[0] + pid_output_pitch - pid_output_roll + pid_output_yaw;
    esc_4 =  1100 + channels[0] - pid_output_pitch - pid_output_roll - pid_output_yaw; 
    
    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;
    if (esc_1 > 1700) esc_1 = 1700;
    if (esc_2 > 1700) esc_2 = 1700; 
    if (esc_3 > 1700) esc_3 = 1700;
    if (esc_4 > 1700) esc_4 = 1700;
    } 
  
  // EMITTING ESC SIGNALS
  TIMER4_BASE->CCR1 = esc_1;
  TIMER4_BASE->CCR2 = esc_2;
  TIMER4_BASE->CCR3 = esc_3;
  TIMER4_BASE->CCR4 = esc_4; 
  TIMER4_BASE->CNT = 5000; 

  serialHandler();
  telemetryHandler();

  // if ((int32_t)micros() - loop_timer > 0) Serial.println((int32_t) micros() - loop_timer);
  while ((int32_t)loop_timer > micros());
  loop_timer = (int32_t) micros() + 4000;
}
