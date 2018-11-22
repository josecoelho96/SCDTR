const int luminaire = 3;
const int lightSensor = A0;

void setup() {
  pinMode(luminaire, OUTPUT);
  pinMode(lightSensor, INPUT);
  Serial.begin(9600);
  analogWrite(luminaire, 0);
}

void loop() {
  
  Serial.println(analogRead(lightSensor));  
  delay(100);
}
