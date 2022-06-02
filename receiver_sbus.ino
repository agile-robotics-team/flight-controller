#define SBUS_STARTBYTE 0x0F
#define SBUS_ENDBYTE   0x00

/*void loop() {
  
  sbusProcess();
  
  Serial.print(channels[0]);
  Serial.print("\t");
  Serial.print(channels[1]);
  Serial.print("\t");
  Serial.print(channels[2]);
  Serial.print("\t");
  Serial.print(channels[3]);
  Serial.print("\t");
  Serial.print(channels[4]);
  Serial.print("\t");
  Serial.print(channels[5]);
  Serial.print("\t");
  Serial.print(channels[6]);
  Serial.print("\t");
  Serial.print(channels[7]);
  Serial.print("\t");
  Serial.print(channels[8]);
  Serial.print("\t");
  Serial.print(channels[9]);
  Serial.print("\t");
  Serial.print(channels[11]);
  Serial.print("\t");
  Serial.println(channels[12]);
}*/

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
    }
  }
}
