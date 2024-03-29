static boolean     new_data = false;
char        serial_rx_buffer[32];
char        serial_tx_buffer[32];
const byte  serial_rx_buffer_max_len = 32;
static byte byte_counter = 0;
char        end_marker = '\n';
char        read_byte;

void serialHandler() {
  serialRead();
  if(new_data == true){

    // GET MODE
    if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'M'){
      Serial.print("{\"GM\":");
      Serial.print(mode_status);
      Serial.println("}"); }
    
    // GET ANGLES
    else if(serial_rx_buffer[0] == 'P' && serial_rx_buffer[1] == 'D'){
      Serial.print("{\"PD\":");
      Serial.print(angle_pitch_output);
      Serial.println("}"); }
    else if(serial_rx_buffer[0] == 'R' && serial_rx_buffer[1] == 'D'){
      Serial.print("{\"RD\":");
      Serial.print(angle_roll_output);
      Serial.println("}"); }

    // GET ESC 
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'E' && serial_rx_buffer[2] == '1') {
      Serial.print("{\"GE1\":");
      Serial.print(esc_1);
      Serial.println("}"); }
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'E' && serial_rx_buffer[2] == '2') {
      Serial.print("{\"GE2\":");
      Serial.print(esc_2);
      Serial.println("}"); }
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'E' && serial_rx_buffer[2] == '3') {
      Serial.print("{\"GE3\":");
      Serial.print(esc_3);
      Serial.println("}"); }
     else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'E' && serial_rx_buffer[2] == '4') {
      Serial.print("{\"GE4\":");
      Serial.print(esc_4);
      Serial.println("}");}
 
    // EEPROM READ PID
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'P') {
      Serial.print("{\"GPP\":");
      Serial.print(EEPROM.read(0));
      Serial.println("}");}
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'I') {
      Serial.print("{\"GPI\":");
      Serial.print(EEPROM.read(1));
      Serial.println("}");}
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'D') {
      Serial.print("{\"GPD\":");
      Serial.print(EEPROM.read(2));
      Serial.println("}");}
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'P') {
      Serial.print("{\"GRP\":");
      Serial.print(EEPROM.read(3));
      Serial.println("}");}
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'I') {
      Serial.print("{\"GRI\":");
      Serial.print(EEPROM.read(4));
      Serial.println("}");}
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'D') {
      Serial.print("{\"GRD\":");
      Serial.print(EEPROM.read(5));
      Serial.println("}");}
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'Y' && serial_rx_buffer[2] == 'P') Serial.println(EEPROM.read(6));
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'Y' && serial_rx_buffer[2] == 'I') Serial.println(EEPROM.read(7));
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'Y' && serial_rx_buffer[2] == 'D') Serial.println(EEPROM.read(8));
    
    // AFTER THIS LINE NEEDS DEBUG MODE
    // ESC SET
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'E' && serial_rx_buffer[2] == '1' && mode_status == 1) esc_1 = 1000 + atoi(get_value_char(serial_rx_buffer));
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'E' && serial_rx_buffer[2] == '2' && mode_status == 1) esc_2 = 1000 + atoi(get_value_char(serial_rx_buffer));
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'E' && serial_rx_buffer[2] == '3' && mode_status == 1) esc_3 = 1000 + atoi(get_value_char(serial_rx_buffer));
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'E' && serial_rx_buffer[2] == '4' && mode_status == 1) esc_4 = 1000 + atoi(get_value_char(serial_rx_buffer));

    // EEPROM WRITE PID
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'P' && mode_status == 1){
      EEPROM.write(0, atoi(get_value_char(serial_rx_buffer)));
      pid_p_gain_pitch = (float)(EEPROM.read(0) / 100.0);
      }
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'I' && mode_status == 1){
      EEPROM.write(1, atoi(get_value_char(serial_rx_buffer)));
      pid_i_gain_pitch = (float)(EEPROM.read(1) / 100000.0);
      }
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'D' && mode_status == 1){
      EEPROM.write(2, atoi(get_value_char(serial_rx_buffer)));
      pid_d_gain_pitch = (float)(EEPROM.read(2));
      }
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'P' && mode_status == 1){
      EEPROM.write(3, atoi(get_value_char(serial_rx_buffer)));
      pid_p_gain_roll = (float)(EEPROM.read(3) / 100.0);
      }
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'I' && mode_status == 1){
      EEPROM.write(4, atoi(get_value_char(serial_rx_buffer)));
      pid_i_gain_roll = (float)(EEPROM.read(4) / 100000.0);
      }
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'D' && mode_status == 1){
      EEPROM.write(5, atoi(get_value_char(serial_rx_buffer)));
      pid_d_gain_roll = (float)(EEPROM.read(5));
      }
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'Y' && serial_rx_buffer[2] == 'P' && mode_status == 1) EEPROM.write(6, atoi(get_value_char(serial_rx_buffer)));
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'Y' && serial_rx_buffer[2] == 'I' && mode_status == 1) EEPROM.write(7, atoi(get_value_char(serial_rx_buffer)));
    else if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'Y' && serial_rx_buffer[2] == 'D' && mode_status == 1) EEPROM.write(8, atoi(get_value_char(serial_rx_buffer)) / 100000);
    
    // RETURN ERRORS BACK
    else Serial.println(serial_rx_buffer);
  
  new_data = false;
  }
}

char * get_value_char( const char *s)
{   
    size_t n = strlen( s ) - 3;
    size_t length = strlen( s );
    return ( char * )( length < n ? s : s + length - n );
}

void serialRead() {
  while (Serial.available() > 0 && new_data == false){
    read_byte = Serial.read();
    if (read_byte != end_marker) {
      serial_rx_buffer[byte_counter] = read_byte;
      byte_counter++;
      if (byte_counter >= serial_rx_buffer_max_len) byte_counter = serial_rx_buffer_max_len - 1;}
    else {
    serial_rx_buffer[byte_counter] = '\0';
    byte_counter = 0;
    if (serial_rx_buffer[0] == 'c' && serial_rx_buffer[1] == 'o' && serial_rx_buffer[2] == 'd' && serial_rx_buffer[3] == 'e') new_data = false;
    else new_data = true;
       }
    }
}
