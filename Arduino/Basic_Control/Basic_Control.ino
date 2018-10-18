// Arduino code for smart luminaire
// SCDTR 1S 2018/19
// Tecnico Lisboa
// David Teles - Jose Coelho - Afonso Soares
// ALL RIGHTS RESERVED

// Config/parameters
// #define DEBUG
#define LOOP_INFO

#define LOW_LUX 100
#define HIGH_LUX 200

const int Vref = 5; // ADC referece voltage
const int R1ref = 9850; // R1 measured value
const long int maxResistanceLdr = 1000000; //1MOhm, defined by datasheet
const float mLdr = -0.652; // LDR characteristic curve: m parameter
const float bLdr = 1.76; // LDR characteristic curve: b parameter

const double kp = 100;//Proportinal
const double ki = 0.5;//Integral
const double kd = 0.0;//Derivative

double iterm = 0;
double dterm = 0;
double pterm = 0;
float lastlux = 0;
double lastoutput = 0;

// Define pins
const int luminaire = 3;
const int presenceSensor = 8;
const int lightSensor = A0;

float measuredLux;
bool occupied;

int brightness; // Current led brightness


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
    Serial.print("pTerm]: ");
    Serial.println(pterm);
    Serial.print("iTerm: ");
    Serial.println(iterm);
    Serial.print("dTerm: ");
    Serial.println(dterm);
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




float updateLEDBrightness(float currentLux, bool occupied) {
  
  double error;
  double output;

  if (occupied == true){
    error=(HIGH_LUX-currentLux)/HIGH_LUX;
  } else {
    error=(LOW_LUX-currentLux)/LOW_LUX;
  }


  //Proportinal Term calculation
  pterm = error * kp;
  
  //Integral Term calculation
  iterm += (error*ki);
  
  if(iterm>100){
    iterm=100;
  } else if(iterm<-100){
    iterm=-100;
  } 


  //Derivative term calculation
  dterm = kd * (lastlux-currentLux);
  
  if (occupied == true){
    
    output = HIGH_LUX * (1+pterm+iterm+dterm);
  
  } else {
    output = LOW_LUX * (1+pterm+iterm+dterm);
    
  }

  output = luxToPWM(output);
  
  analogWrite(luminaire,output);
  lastlux = currentLux;
  return map(output,0,255,0,100);
}


float luxToPWM(double lux){
  float pwm;
  pwm = lux/2;
  return pwm;
}



