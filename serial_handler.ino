boolean     new_data = false;
char        serial_rx_buffer[32];
const byte  serial_rx_buffer_max_len = 32;
static byte byte_counter = 0;
char        end_marker = '\n';
char        read_byte;

void serialHandler() {
  serialRead();
  if(new_data == true){
    if     (serial_rx_buffer[0] == 'O') loop_status = 1; 
    else if(serial_rx_buffer[0] == 'F') loop_status = 0;
    new_data = false;
  }
}

void serialRead() {
  while (Serial.available() > 0 && new_data == false){
    digitalWrite(LED_BUILTIN, HIGH);
    read_byte = Serial.read();
    if (read_byte != end_marker) {
      serial_rx_buffer[byte_counter] = read_byte;
      byte_counter++;
      if (byte_counter >= serial_rx_buffer_max_len) byte_counter = serial_rx_buffer_max_len - 1;}
    else {
    serial_rx_buffer[byte_counter] = '\0';
    byte_counter = 0;
    
    new_data = true;}}
  digitalWrite(LED_BUILTIN, LOW);
}
