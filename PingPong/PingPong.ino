#include <Wire.h>

// 10 - ACM0
// 11 - ACM1
const byte address = 11;
volatile byte msg;
volatile byte sendNew;

void setup() {
  sendNew = false;
  Serial.begin(9600);
  Serial.println("Begin I2C...");
  Wire.begin(address);
  Wire.onReceive(receiveData);
  TWAR = TWAR | B00000001;

  sendData(0);
}

void loop() {
  if (sendNew == true) {
    sendData(msg);
    sendNew = false;
  }
}

void receiveData(int howMany) {
  while (Wire.available()) {
    msg = Wire.read();
  }
  msg = msg + 1;
  Serial.println(msg);
  delay(1000);
  sendNew = true;
}

void sendData(volatile byte value) {
  Wire.beginTransmission(0);
  Wire.write(value);
  Wire.endTransmission(); 
}
