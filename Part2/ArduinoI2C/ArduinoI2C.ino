// MASTER

#include <Wire.h>

byte x = 1;

void setup() {
  Serial.begin(9600);  // start serial for output
  Wire.begin();
  TWAR = TWAR | B00000001;
}

void loop() {
  Wire.beginTransmission(0);
  Wire.write(x);
  Wire.endTransmission();
  x++;
  delay(1000);
}
