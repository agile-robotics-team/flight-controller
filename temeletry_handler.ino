int angle_respond_delay = 0;

void telemetryHandler(){

  // TELEMETRY BOOT MODE
  if(telemetry_status == 0) digitalWrite(PC13, HIGH);
  
  // TELEMETRY SLOW MODE
  else if(telemetry_status == 1){
    if ((telemetry_delay++ % 170) == 0) digitalWrite(PC13, !digitalRead(PC13));
    
    if (telemetry_delay % 7  == 0){
      angle_respond_delay++;
      
      if     (angle_respond_delay == 1){
      Serial.print("{\"PD\":");
      Serial.print(angle_pitch_output);
      Serial.println("}");}
      
      else if(angle_respond_delay == 2){
      Serial.print("{\"RD\":");
      Serial.print(angle_roll_output);
      Serial.println("}");}
      
      if     (angle_respond_delay == 2) angle_respond_delay = 0;
      
      }

    if (telemetry_delay % 75  == 0){
      Serial.print("{\"TEM1\":");
      Serial.print(temperature);
      Serial.println("}");
    }

      
    /*if (telemetry_delay % 50  == 0){
      Serial.print("{\"CH1\":");
      Serial.print(channels[0]);
      Serial.println("}");
      Serial.print("{\"CH2\":");
      Serial.print(channels[1]);
      Serial.println("}");
      Serial.print("{\"CH3\":");
      Serial.print(channels[2]);
      Serial.println("}");
      }

    if (telemetry_delay % 60  == 0){ 
      Serial.print("{\"CH4\":");
      Serial.print(channels[3]);
      Serial.println("}");
      Serial.print("{\"CH5\":");
      Serial.print(channels[4]);
      Serial.println("}");
      Serial.print("{\"CH6\":");
      Serial.print(channels[5]);
      Serial.println("}");
      }
    
    if (telemetry_delay % 70  == 0){ 
      Serial.print("{\"CH7\":");
      Serial.print(channels[6]);
      Serial.println("}");
      Serial.print("{\"CH8\":");
      Serial.print(channels[7]);
      Serial.println("}");
      Serial.print("{\"CH9\":");
      Serial.print(channels[8]);
      Serial.println("}");
      }

    if (telemetry_delay % 700  == 0){ 
      Serial.print("{\"TEM1\":");
      Serial.print(temperature);
      Serial.println("}");
      }
      
    if (telemetry_delay % 5000  == 0){ 
      Serial.print("{\"GPP\":");
      Serial.print(EEPROM.read(0));
      Serial.println("}");
      Serial.print("{\"GPI\":");
      Serial.print(EEPROM.read(1));
      Serial.println("}");
      Serial.print("{\"GPD\":");
      Serial.print(EEPROM.read(2));
      Serial.println("}");
      Serial.print("{\"GRP\":");
      Serial.print(EEPROM.read(3));
      Serial.println("}");
      Serial.print("{\"GRI\":");
      Serial.print(EEPROM.read(4));
      Serial.println("}");
      Serial.print("{\"GRD\":");
      Serial.print(EEPROM.read(5));
      Serial.println("}");
      }*/
    }
    
  // TELEMETRY FAST MODE 
  else if(telemetry_status == 2){
    if ((mode_blinker++ % 35) == 0) digitalWrite(PC13, !digitalRead(PC13));
    if (telemetry_delay++ > 9){ 
      Serial.print("{\"PD\":");
      Serial.print(angle_pitch_output);
      Serial.print(",\"RD\":");
      Serial.print(angle_roll_output);
      Serial.print(",\"CH\":\"");
      Serial.print(channels[0]);
      Serial.print(",");
      Serial.print(channels[1]);
      Serial.print(",");
      Serial.print(channels[2]);
      Serial.print(",");
      Serial.print(channels[3]);
      Serial.print(",");
      Serial.print(channels[4]);
      Serial.print(",");
      Serial.print(channels[5]);
      Serial.print(",");
      Serial.print(channels[6]);
      Serial.print(",");
      Serial.print(channels[7]);
      Serial.print(",");
      Serial.print(channels[8]);
      Serial.print("\",\"ESC1\":");
      Serial.print(esc_1);
      Serial.print(",\"ESC2\":");
      Serial.print(esc_2);
      Serial.print(",\"ESC3\":");
      Serial.print(esc_3);
      Serial.print(",\"ESC4\":");
      Serial.print(esc_4);
      Serial.print("\",\"TEM1\":");
      Serial.print(temperature);
      Serial.println("}");
      telemetry_delay = 0;
      }
      
    if (esc_1 < 1000) esc_1 = 1000;
    if (esc_2 < 1000) esc_2 = 1000;
    if (esc_3 < 1000) esc_3 = 1000;
    if (esc_4 < 1000) esc_4 = 1000;
    if (esc_1 > 1500) esc_1 = 1500;
    if (esc_2 > 1500) esc_2 = 1500; 
    if (esc_3 > 1500) esc_3 = 1500;
    if (esc_4 > 1500) esc_4 = 1500;

    }
}
