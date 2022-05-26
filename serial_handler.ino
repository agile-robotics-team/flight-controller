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

    // CHANGING MODE 
    if(serial_rx_buffer[0] == 'S' && serial_rx_buffer[1] == 'M'){
      digitalWrite(PC13, HIGH);
      loop_status = atoi(get_value_char(serial_rx_buffer));
      if(loop_status == 1){
        esc_1 = 1100;
        esc_2 = 1100;
        esc_3 = 1100;
        esc_4 = 1100;
        }
       else{
        esc_1 = 1000;
        esc_2 = 1000;
        esc_3 = 1000;
        esc_4 = 1000;
        }
      }

    // GET MODE
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'M'){
      Serial.print("{\"GM\":");
      Serial.print(loop_status);
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

    // GET PID VALUES
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'P') {
      Serial.print("{\"GPP\":");
      Serial.print(pid_p_gain_pitch);
      Serial.println("}"); }
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'I') {
      Serial.print("{\"GPI\":");
      Serial.print(pid_i_gain_pitch);
      Serial.println("}"); }
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'P' && serial_rx_buffer[2] == 'D') {
      Serial.print("{\"GPD\":");
      Serial.print(pid_d_gain_pitch);
      Serial.println("}"); } 
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'P') {
      Serial.print("{\"GRP\":");
      Serial.print(pid_p_gain_roll);
      Serial.println("}"); }
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'I') {
      Serial.print("{\"GRI\":");
      Serial.print(pid_i_gain_roll);
      Serial.println("}"); }
    else if(serial_rx_buffer[0] == 'G' && serial_rx_buffer[1] == 'R' && serial_rx_buffer[2] == 'D') {
      Serial.print("{\"GRD\":");
      Serial.print(pid_d_gain_roll);
      Serial.println("}"); }
      
    // THIS LINES NEEDS DEBUG MODE
    // ESC SET
    else if(serial_rx_buffer[0] == 'E' && serial_rx_buffer[1] == '1' && loop_status == 2) esc_1 = 1000 + atoi(get_value_char(serial_rx_buffer));
    else if(serial_rx_buffer[0] == 'E' && serial_rx_buffer[1] == '2' && loop_status == 2) esc_2 = 1000 + atoi(get_value_char(serial_rx_buffer));
    else if(serial_rx_buffer[0] == 'E' && serial_rx_buffer[1] == '3' && loop_status == 2) esc_3 = 1000 + atoi(get_value_char(serial_rx_buffer));
    else if(serial_rx_buffer[0] == 'E' && serial_rx_buffer[1] == '4' && loop_status == 2) esc_4 = 1000 + atoi(get_value_char(serial_rx_buffer));

    // RETURN ERRORS BACK
    else Serial.println(serial_rx_buffer);
  
  new_data = false;
  }
}















char * get_value_char( const char *s)
{   
    size_t n = strlen( s ) - 2;
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
    new_data = true;}}
}
