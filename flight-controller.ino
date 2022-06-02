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

int     loop_status = 0;
int     mode_blinker = 0;
int     debug_mode_delay = 0;

long  acc_total_vector;
float angle_pitch = 0, angle_roll = 0;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output, angle_yaw_output;

float pid_error_temp;
float pid_i_mem_pitch, pid_pitch_setpoint = 0, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_roll,  pid_roll_setpoint = 0,  gyro_roll_input,  pid_output_roll,  pid_last_roll_d_error;

double pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch;
double pid_p_gain_roll, pid_i_gain_roll,pid_d_gain_roll;

int   pid_max_roll  =  400; //Maximum output of the PID-controller (+/-).
int   pid_max_pitch =  400; //Maximum output of the PID-controller (+/-).

const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_READ_ADDRESS = 0x3B;
int           esc_1 = 1000, esc_2 = 1000, esc_3 = 1000, esc_4 = 1000;

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
  
  pid_p_gain_pitch = EEPROM.read(0) / 100.0;
  pid_i_gain_pitch = EEPROM.read(1) / 100000.0;
  pid_d_gain_pitch = EEPROM.read(2);
  pid_p_gain_roll  = EEPROM.read(3) / 100.0;
  pid_i_gain_roll  = EEPROM.read(4) / 100000.0;
  pid_d_gain_roll  = EEPROM.read(5);}

void loop() {

  // PREPARING SYSTEM FOR CURRENT LOOP
  serialHandler();
  serialLinkHandler();
  calcAngles();
  sbusProcess();
  
  // STABILIZE MODE
  if(loop_status == 1){
    calcPid();  
    esc_1 = 150 - pid_output_pitch + pid_output_roll;
    esc_2 = 150 + pid_output_pitch + pid_output_roll;
    esc_3 = 150 + pid_output_pitch - pid_output_roll;
    esc_4 = 150 - pid_output_pitch - pid_output_roll; 

    if (debug_mode_delay++ > 2){ 
      Serial.print("{\"PD\":");
      Serial.print(angle_pitch_output);
      Serial.print(",\"RD\":");
      Serial.print(angle_roll_output);
      Serial.print(",\"ESC1\":");
      Serial.print(esc_1);
      Serial.print(",\"ESC2\":");
      Serial.print(esc_2);
      Serial.print(",\"ESC3\":");
      Serial.print(esc_3);
      Serial.print(",\"ESC4\":");
      Serial.print(esc_4);
      Serial.println("}");
      debug_mode_delay = 0;}
    
    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;
    if (esc_1 > 1500) esc_1 = 1500;
    if (esc_2 > 1500) esc_2 = 1500; 
    if (esc_3 > 1500) esc_3 = 1500;
    if (esc_4 > 1500) esc_4 = 1500;}

  else if(loop_status == 4){
      Serial.print("{\"CH1\":");
      Serial.print(channels[0]);
      Serial.print(",\"CH2\":");
      Serial.print(channels[1]);
      Serial.print(",\"CH3\":");
      Serial.print(channels[2]);
      Serial.print(",\"CH4\":");
      Serial.print(channels[3]);
      Serial.print(",\"CH5\":");
      Serial.print(channels[4]);
      Serial.print(",\"CH6\":");
      Serial.print(channels[5]);
      Serial.print(",\"CH7\":");
      Serial.print(channels[6]);
      Serial.print(",\"CH8\":");
      Serial.print(channels[7]);
      Serial.println("}");
    }

  // SKIP IF OTHER MODES
  else if(loop_status == 2 || loop_status == 3);
  
  // BOOT MODE
  else {
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;}

  // EMITTING ESC SIGNALS
  TIMER4_BASE->CCR1 = 1000 + esc_1;
  TIMER4_BASE->CCR1 = 1000 + esc_2;
  TIMER4_BASE->CCR1 = 1000 + esc_3;
  TIMER4_BASE->CCR1 = 1000 + esc_4;
 
  while (loop_timer > micros());
  loop_timer = micros() + 4000;
}
