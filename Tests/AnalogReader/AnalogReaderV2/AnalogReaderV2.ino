#define MAX 309

unsigned int adcMeasurement[MAX];
unsigned long int adcMeasurementTimestamp[MAX];
int currentMeasurement = 0;
const int sensor = A0;

void setup() {
  Serial.begin(2000000);
  pinMode(sensor, INPUT);
  currentMeasurement = 0;
}

void loop() {
  while (currentMeasurement < MAX) {
    adcMeasurementTimestamp[currentMeasurement] = micros();
    adcMeasurement[currentMeasurement] = analogRead(A0);
    currentMeasurement++;
  }
  for (int i = 0; i < MAX; i++) {
    Serial.print(adcMeasurementTimestamp[i]);
    Serial.print("\t");
    Serial.println(adcMeasurement[i]);
  }
  currentMeasurement = 0;
}
