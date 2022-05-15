#include <Wire.h>
TwoWire HWire(2, I2C_FAST_MODE);
int loop_timer;

void setup() {
  // init communcation ways
  Serial.begin(115200);
  HWire.setClock(400000);
  HWire.begin();
  delay(250);

  // init drivers
  initIMU();
  initEscDriver();
  calibrateGyro();
}

void loop() {

  // get angles from imu
  calcAngles();

  // set pwm signals 1000-2000
  TIMER4_BASE -> CCR1 = 1000;
  TIMER4_BASE -> CCR2 = 1000;

  // set main loop frequency 250hz
  while (loop_timer > micros());
  loop_timer = micros() + 4000;
}
