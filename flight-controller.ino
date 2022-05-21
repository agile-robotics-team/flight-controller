#include <Wire.h>
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

long  acc_total_vector;
float angle_pitch = 0, angle_roll = 0;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

float pid_error_temp;
float pid_i_mem_pitch, pid_pitch_setpoint = 0, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_p_gain_pitch = 0.61;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = 0.00018; //0.04;   //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = 100; //18.0;   //Gain setting for the pitch D-controller.
int   pid_max_pitch =  500;          //Maximum output of the PID-controller (+/-).

const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_READ_ADDRESS = 0x3B;
int           esc_1, esc_2;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
//KalmanFilter kalmanY(0.001, 0.003, 0.03);
float kalPitch = 0;
//float kalRoll = 0;

void setup() {
  // init communcation ways
  Serial.begin(115200);
  HWire.setClock(400000);
  HWire.begin();
  delay(250);

  // init drivers
  initIMU();
  initEscDriver();
  calibrateGyro();
  
  //delay(3000);
  //gyro_x_cal= -5371;
  //gyro_y_cal= -258;
  //gyro_z_cal= -102;
       
}

void loop() {

  serialHandler();
  TIMER4_BASE->ARR = 5000;
  calcAngles();

  if(loop_status == 1){
    calcPid();  
    esc_1 = 1100 + 125 - pid_output_pitch;
    esc_2 = 1100 + 125 + pid_output_pitch;
    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_1 > 2000) esc_1 = 2000;
    if (esc_2 > 2000) esc_2 = 2000;
    }

  else {
    esc_1 = 1000;
    esc_2 = 1000;
    }
    
  TIMER4_BASE -> CCR1 = esc_1;
  TIMER4_BASE -> CCR2 = esc_2;
  Serial.print(angle_pitch_output);
  Serial.print("\t");
  Serial.println(loop_status);
  while (loop_timer > micros());
  loop_timer = micros() + 4000;
}
