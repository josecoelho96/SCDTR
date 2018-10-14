const int sensor = A0;

void setup() {
  Serial.begin(2000000);
  pinMode(sensor, INPUT);
  ADCSRA &= ~(1 << ADPS2);
  ADCSRA &= ~(1 << ADPS1);
  ADCSRA &= ~(1 << ADPS0);
  ADCSRA |= (1 << ADPS2); // 16 prescaller => fs = 76.9 kHz
}

void loop() {
  Serial.print(micros());
  Serial.print("\t");
  Serial.println(analogRead(sensor));
}
