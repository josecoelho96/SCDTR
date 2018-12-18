// MULTI MASTER

#include <Wire.h>

byte x = 0;
byte address = 2;

void setup() {
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  TWAR = TWAR | B00000001;
  Serial.begin(9600);
}

void loop() {
  Wire.beginTransmission(0);
  Wire.write(address);
  Wire.write(x);
  Wire.endTransmission();
  x++;
  delay(1000);
}

void receiveEvent(int howMany) {
  while (Wire.available()) {
    byte rx = Wire.read();
    Serial.println(rx);
  }
}
