const int luminairePin = 3;

void setup() {
  pinMode(luminairePin, OUTPUT);
}

void loop() {
  analogWrite(luminairePin, 0);
  delay(1000);

  analogWrite(luminairePin, 150);
  delay(1000);

  analogWrite(luminairePin, 255);
  delay(1000);
}
