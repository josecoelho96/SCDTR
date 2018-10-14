const int sensor = A0;

void setup() {
  Serial.begin(2000000);
  pinMode(sensor, INPUT);
}

void loop() {
  Serial.print(micros());
  Serial.print("\t");
  Serial.println(analogRead(sensor));
}
