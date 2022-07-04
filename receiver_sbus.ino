#define SBUS_STARTBYTE 0x0F
#define SBUS_ENDBYTE   0x00

void sbusProcess() {
  static byte buffer[25];
  static byte buffer_index = 0;

  while (Serial2.available()) {
    byte rx = Serial2.read();
    if (buffer_index == 0 && rx != SBUS_STARTBYTE) {
      //incorrect start byte, out of sync
      continue;
    }

    buffer[buffer_index++] = rx;

    if (buffer_index == 25) {
      buffer_index = 0;
      if (buffer[24] != SBUS_ENDBYTE) {
        //incorrect end byte, out of sync
        continue;
      }

      channels[0]  = ((buffer[1]    |buffer[2]<<8)                 & 0x07FF);
      channels[1]  = ((buffer[2]>>3 |buffer[3]<<5)                 & 0x07FF);
      channels[2]  = ((buffer[3]>>6 |buffer[4]<<2 |buffer[5]<<10)  & 0x07FF);
      channels[3]  = ((buffer[5]>>1 |buffer[6]<<7)                 & 0x07FF);
      channels[4]  = ((buffer[6]>>4 |buffer[7]<<4)                 & 0x07FF);
      channels[5]  = ((buffer[7]>>7 |buffer[8]<<1 |buffer[9]<<9)   & 0x07FF);
      channels[6]  = ((buffer[9]>>2 |buffer[10]<<6)                & 0x07FF);
      channels[7]  = ((buffer[10]>>5|buffer[11]<<3)                & 0x07FF);
      channels[8]  = ((buffer[12]   |buffer[13]<<8)                & 0x07FF);
      channels[9]  = ((buffer[13]>>3|buffer[14]<<5)                & 0x07FF);
      channels[10] = ((buffer[14]>>6|buffer[15]<<2|buffer[16]<<10) & 0x07FF);
      channels[11] = ((buffer[16]>>1|buffer[17]<<7)                & 0x07FF);
      channels[12] = ((buffer[17]>>4|buffer[18]<<4)                & 0x07FF);
      channels[13] = ((buffer[18]>>7|buffer[19]<<1|buffer[20]<<9)  & 0x07FF);
      channels[14] = ((buffer[20]>>2|buffer[21]<<6)                & 0x07FF);
      channels[15] = ((buffer[21]>>5|buffer[22]<<3)                & 0x07FF);
      ((buffer[23])      & 0x0001) ? channels[16] = 2047: channels[16] = 0;
      ((buffer[23] >> 1) & 0x0001) ? channels[17] = 2047: channels[17] = 0;

      channels[0] = map(channels[0], 180, 1820, 0, 1000);
      channels[1] = map(channels[1], 180, 1820, 0, 1000);
      channels[2] = map(channels[2], 180, 1820, 0, 1000);
      channels[3] = map(channels[3], 180, 1820, 0, 1000);
      channels[4] = map(channels[4], 180, 1820, 0, 1000);
      channels[5] = map(channels[5], 180, 1820, 0, 1000);
      channels[6] = map(channels[6], 172, 1820, 0, 1000);
      channels[7] = map(channels[7], 172, 1820, 0, 1000);
      channels[8] = map(channels[8], 180, 1820, 0, 1000);
      
      if (channels[0] < 0) channels[0] = 0;
      if (channels[1] < 0) channels[1] = 0;
      if (channels[2] < 0) channels[2] = 0;
      if (channels[3] < 0) channels[3] = 0;
      if (channels[4] < 0) channels[4] = 0;
      if (channels[5] < 0) channels[5] = 0;
      if (channels[6] < 0) channels[6] = 0;
      if (channels[7] < 0) channels[7] = 0;
      if (channels[8] < 0) channels[8] = 0;

      //pid_p_gain_pitch = (float)channels[6] / 200.0;
      //pid_d_gain_pitch = (float)channels[7] / 10.0;
      //pid_p_gain_roll = pid_p_gain_pitch;
      //pid_d_gain_roll = pid_d_gain_pitch;
    }
  }

  mode_status = 0;
  if (channels[4] < 300) mode_status = 0;
  else if ((channels[4] > 300) && channels[4] < 700) mode_status = 1;
  else if (channels[4] > 700) mode_status = 2;
  else mode_status = 0;
  
  if (channels[5] < 300) telemetry_status = 0;
  else if ((channels[5] > 300) && channels[5] < 700) telemetry_status = 1;
  else telemetry_status = 2;
}
