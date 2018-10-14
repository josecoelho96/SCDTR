#define MAX 340

byte adcMeasurement[MAX];
unsigned long int adcMeasurementTimestamp[MAX];
int currentMeasurement = 0;
const int sensor = A0;
void setup() {
  Serial.begin(2000000);
  currentMeasurement = 0;
  ADCSRA = B00000000; // Clear ADCSRA register
  ADCSRB = B00000000; // Clear ADCSRB register
  ADMUX = B00000000; // Clear ADMUX register
  // Set ADC0 (A0): Done when setting ADMUX(3:0) = 0000
  ADMUX |= (1 << REFS0); // Set reference voltage to VCC
  ADMUX |= (1 << ADLAR); // Left align ADC value
  ADCSRA |= (1 << ADPS2); // 16 prescaller => fs = 76.9 kHz
  ADCSRA |= (1 << ADATE); // Enable auto trigger
  ADCSRA |= (1 << ADIE); // Enable interrupts
  ADCSRA |= (1 << ADEN); // Enable ADC
  ADCSRA |= (1 << ADSC); // Start measurements
}

void loop() {
  if (currentMeasurement == MAX) {
    for (int i = 0; i < MAX; i++) {
      Serial.print(adcMeasurementTimestamp[i]);
      Serial.print("\t");
      Serial.println(adcMeasurement[i]);
    }
    currentMeasurement = 0;
  }
}

ISR(ADC_vect) {
  if (currentMeasurement < MAX) {
    adcMeasurementTimestamp[currentMeasurement] = micros();
    adcMeasurement[currentMeasurement] = ADCH;
    currentMeasurement++;
  }
}
