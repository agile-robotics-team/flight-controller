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

// receiver control
volatile int channels[18];
TwoWire HWire(PB7, PB6);

void setup() {
  // init communcation ways
  
  pinMode(PC13, OUTPUT);

  Serial.begin(115200);
  Serial.println("Serial1 Başlatıldı.");

  
  HWire.setClock(400000);
  HWire.begin();
  delay(250);
  Serial.println("Wire Başlatıldı.");
  delay(250);

  // init drivers
  Serial.println("Başlatılıyor...");
  initIMU();
  Serial.println("IMU Başlatıldı.");
  initEscDriver();
  Serial.println("ESC'ler Başlatıldı.");
  calibrateGyro();
  Serial.println("Gyro kalibre edildi.");
  digitalWrite(PC13, HIGH);
  
  //delay(3000);
  //gyro_x_cal= -5371;
  //gyro_y_cal= -258;
  //gyro_z_cal= -102;
       
}

void loop() {

  // PREPARING SYSTEM
  serialHandler();
  serialLinkHandler();
  calcAngles();
  // sbusProcess();
  
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
    Serial.print(pid_p_gain_pitch);
    Serial.print("\t");
    Serial.print(pid_d_gain_pitch);
    Serial.print("\t");
    Serial.println(pid_i_gain_pitch*100000);
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
  esc_1 = map(esc_1, 0, 255, 0, 1000);
  esc_2 = map(esc_2, 0, 255, 0, 1000);
  esc_3 = map(esc_3, 0, 255, 0, 1000);
  esc_4 = map(esc_4, 0, 255, 0, 1000);
  analogWrite(D12, esc_1); 
  analogWrite(D13, esc_2);
  analogWrite(D14, esc_3); 
  analogWrite(D15, esc_4);
  
  while (loop_timer > micros());
  loop_timer = micros() + 4000;
}
