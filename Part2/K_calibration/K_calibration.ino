const int luminaire = 3;
const int lightSensor = A0;


void setup() {
  pinMode(luminaire, OUTPUT);
  pinMode(lightSensor, INPUT);
}

void loop() {
  analogWrite(luminaire, 255);
  delay(3000);
  analogWrite(luminaire, 0);
  delay(3000);
}
