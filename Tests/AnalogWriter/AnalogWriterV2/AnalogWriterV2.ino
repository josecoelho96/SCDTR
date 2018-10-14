const int luminairePin = 3;

void setup() {
  pinMode(luminairePin, OUTPUT);
  // Reset PWM speed on pin 3 and 11
  TCCR2B &= B11111000;
  TCCR2B |= (1 << CS22);
  // TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);
}

void loop() {
  analogWrite(luminairePin, 0);
  delay(1000);

  analogWrite(luminairePin, 150);
  delay(1000);

  analogWrite(luminairePin, 255);
  delay(1000);
}
