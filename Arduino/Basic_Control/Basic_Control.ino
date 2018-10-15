// Arduino code for smart luminaire
// SCDTR 1S 2018/19
// Tecnico Lisboa
// David Teles - Jose Coelho - Afonso Soares
// ALL RIGHTS RESERVED

// Config/parameters
// #define DEBUG
#define LOOP_INFO

#define LOW_LUX 200
#define HIGH_LUX 500

const int Vref = 5; // ADC referece voltage
const int R1ref = 9850; // R1 measured value
const long int maxResistanceLdr = 1000000; //1MOhm, defined by datasheet
const float mLdr = -0.652; // LDR characteristic curve: m parameter
const float bLdr = 1.76; // LDR characteristic curve: b parameter


// Define pins
const int luminaire = 3;
const int presenceSensor = 8;
const int lightSensor = A0;

float measuredLux;
bool occupied;

int brightness; // Current led brightness
/*
float lastLux;
*/
void setup() {
  Serial.begin(2000000);

  // Define IO
  pinMode(luminaire, OUTPUT);
  pinMode(lightSensor, INPUT);
  pinMode(presenceSensor, INPUT_PULLUP);
}

void loop() {

  // get inputs
  measuredLux = getLDRLux();
  // negate to use internal pull up resistors
  occupied = !digitalRead(presenceSensor);

  // calculate new brightness for the LED
  brightness = updateLEDBrightness(measuredLux, occupied);

  #ifdef LOOP_INFO
    Serial.print("Luminance [Lux]: ");
    Serial.println(measuredLux);
    Serial.print("Occupied [Boolean]: ");
    Serial.println(occupied);
    Serial.print("Brightness [%]: ");
    Serial.println(brightness);
  #endif

  delay(1000);
}


float getLDRLux() {
  int adcLdr;
  float voltageLdr;
  float resistanceLdr;
  float luxLdr;

  adcLdr = analogRead(lightSensor);
  #ifdef DEBUG
    Serial.print("LDR [ADC value]: ");
    Serial.println(adcLdr);
  #endif

  voltageLdr = (Vref*adcLdr)/1023.0;
  #ifdef DEBUG
    Serial.print("LDR [Voltage at terminal]: ");
    Serial.println(voltageLdr);
  #endif

  if (voltageLdr == 0) {
    resistanceLdr = maxResistanceLdr;
  } else {
    resistanceLdr = R1ref*(Vref/voltageLdr - 1);
  }

  #ifdef DEBUG
    Serial.print("LDR [Resistance]: ");
    Serial.println(resistanceLdr);
  #endif

  // Resistance is converted to kOhm
  luxLdr = pow(10, (log10(resistanceLdr/1000) - bLdr)/mLdr);

  #ifdef DEBUG
    Serial.print("LDR [Luminance]: ");
    Serial.println(luxLdr);
  #endif

  return luxLdr;
}


int updateLEDBrightness(float currentLux, bool occupied) {
  float p = 1.0; // Proportional
  int luxError;
  int output;

  if (occupied == true) {
    luxError = HIGH_LUX - currentLux;
  } else {
    luxError = LOW_LUX - currentLux;
  }

  if (brightness != 0) {
     output = brightness * (1+(luxError/100)*p);
  } else if (luxError < 0){
    output = 0;
  } else {
    output = 1 * (1+(luxError/100));
  }

  if (output > 100) {
    output = 100;
  } else if (output < 0) {
    output = 0;
  }

  analogWrite(luminaire, map(output, 0, 100, 0, 255));
  return output;
}
