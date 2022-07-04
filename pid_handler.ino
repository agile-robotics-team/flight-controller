void calcPid() {
  
  //Setpoint Calc
  pitch_level_adjust = angle_pitch_output * 15;
  roll_level_adjust  = angle_roll_output  * 15; 
   
  if (channels[2] < 490) pid_pitch_setpoint = -1*(channels[2] - 490);
  else if (channels[2] > 510) pid_pitch_setpoint = -1*(channels[2] - 510);
  else pid_pitch_setpoint = 0;
  
  if (channels[1] < 490) pid_roll_setpoint = channels[1] - 490;
  else if (channels[1] > 510) pid_roll_setpoint = channels[1] - 510;
  else pid_roll_setpoint  = 0;
   
  if (channels[3] < 490) pid_yaw_setpoint = (channels[3] - 490) / 3;
  else if (channels[3] > 510) pid_yaw_setpoint = (channels[3] - 510) / 3;
  else pid_yaw_setpoint   = 0;

  pid_pitch_setpoint += pitch_level_adjust;
  pid_roll_setpoint  += roll_level_adjust;
  pid_pitch_setpoint /= 3.0;
  pid_roll_setpoint  /= 3.0;

  /*if (channels[0] > 60) {
    if      (channels[3] > 510) pid_yaw_setpoint = (channels[3] - 510) / 3.0;
    else if (channels[3] < 490) pid_yaw_setpoint = (channels[3] - 490) / 3.0;
  }*/
  
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
  pid_last_pitch_d_error = pid_error_temp;

  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1; 
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1; 
  pid_last_roll_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
  pid_last_yaw_d_error = pid_error_temp;
  
}
