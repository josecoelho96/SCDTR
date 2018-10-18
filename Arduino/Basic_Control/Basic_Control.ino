// Arduino code for smart luminaire
// SCDTR 1S 2018/19
// Tecnico Lisboa
// David Teles - Jose Coelho - Afonso Soares
// ALL RIGHTS RESERVED

// Config/parameters
// #define DEBUG
//#define LOOP_INFO

#define LOW_LUX 50
#define HIGH_LUX 120

const int Vref = 5; // ADC referece voltage
const int R1ref = 9850; // R1 measured value
const long int maxResistanceLdr = 1000000; //1MOhm, defined by datasheet
const float mLdr = -0.652; // LDR characteristic curve: m parameter
const float bLdr = 1.76; // LDR characteristic curve: b parameter




const double tau = 1;
const double a = 1/tau;
const double b = 1;
const double T = 1;
const double kp = 1;//Proportinal
const double ki = 0.5;//Integral
const double kd = 0.0;//Derivative

double iterm = 0;
double dterm = 0;
double pterm = 0;
float lastlux = 0;
double lasterror = 0;
double lastoutput = 0;

// Define pins
const int luminaire = 3;
const int presenceSensor = 8;
const int lightSensor = A1;

float measuredLux;
bool occupied;

int brightness; // Current led brightness


void setup() {
  Serial.begin(2000000);
  // Define IO
  pinMode(luminaire, OUTPUT);
  pinMode(lightSensor, INPUT);
  pinMode(presenceSensor, INPUT_PULLUP);
  
  // Enable faster PWM on Pin 3 (and 11);
  // Reset TCCR2B register Clock Select (CS) bits
  // TC2 - Timer/Counter1 (8 bits)
  // TCCR2B - TC2 Control Register B
  TCCR2B &= B11111000;
  // Setting no prescaler: 001
  // TCCR2B |= (1 << CS22);
  // TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);


  
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
    delay(1000);
  #endif

  
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
  int ref;
  double error;
  double output;

  if (occupied == true){
    ref = HIGH_LUX;
  } else {
    ref = LOW_LUX;
  }


  error=(ref-currentLux);
  
  //Proportinal Term calculation
  pterm = ref * kp * b - kp * currentLux;
  
  //Integral Term calculation
  iterm += ((error + lasterror)* ki * kp)*(T/2);
  
//  if(iterm>100){
//    iterm=100;
//  } else if(iterm<-100){
//    iterm=-100;
//  } 


  //Derivative term calculation
  dterm = kd/(kd+a*T) * lastlux - kp*kd*a/(kd+a*T)*(currentLux-lastlux);
 

  output = (pterm+iterm+dterm);
  if (output <0){
    output = 0;
  }

  lastoutput = output;
  output = luxToPWM(output);
  
  analogWrite(luminaire,output);
  lastlux = currentLux;
  lasterror = error;
  return map(output,0,255,0,100);
}


float luxToPWM(double lux){
  float pwm;
  pwm = lux;
  if(pwm > 255){
    pwm = 255;
  } else if(pwm < 0){
    pwm = 0;
  }
  return pwm;
}



