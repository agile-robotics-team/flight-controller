void initEscDriver(){
  
  pinMode(D12, OUTPUT);
  pinMode(D13, OUTPUT);
  pinMode(D14, OUTPUT);
  pinMode(D15, OUTPUT);

  analogWriteFrequency(250);
  analogWrite(D12, 0); 
  analogWrite(D13, 0);
  analogWrite(D14, 0); 
  analogWrite(D15, 0); 
}
  
