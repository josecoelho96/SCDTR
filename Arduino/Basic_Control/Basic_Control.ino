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
#define A_LUXTOPWM 1
#define B_LUXTOPWM 0




const int Vref = 5; // ADC referece voltage
const int R1ref = 9850; // R1 measured value
const long int maxResistanceLdr = 1000000; //1MOhm, defined by datasheet
const float mLdr = -0.652; // LDR characteristic curve: m parameter
const float bLdr = 1.76; // LDR characteristic curve: b parameter



//Contants used by the PID controle
const double tau = 1;
const double a = 1/tau;
const double b = 1;
const double T = 1;
const double kp = 1;//Proportinal
const double ki = 0.5;//Integral
const double kd = 0.0;//Derivative

//Values used by the PID controle
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


//Variables used to keep track of the current state
float measuredLux; //Lux measured in the room
bool occupied; //Bollean state that represents if the place that the luminaire is lighting if occupied or not
int brightness; // Current led brightness [0-255]


void setup() {
  Serial.begin(2000000);
  // Define IO
  pinMode(luminaire, OUTPUT);
  pinMode(lightSensor, INPUT);
  pinMode(presenceSensor, INPUT_PULLUP);
  occupied = !digitalRead(presenceSensor);
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

  // calculate new brightness for the LED
  feedBack();

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

void feedForward(){
  int ref;
  int output;

  //Defines what is de disered Lux in the room depending on the current occupation state
  if (occupied == true){
    ref = HIGH_LUX;
  } else {
    ref = LOW_LUX;
  }

  output = luxToPWM(ref); //Relationship beetween lux and the pwm value to achieve the proper output

  analogWrite(luminaire,output);
  brightness = output;
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




float feedBack() {
  measureLux = getLDRLux();
  int ref;
  double error;
  double output;

  //Defines what is de disered Lux in the room depending on the current occupation state
  if (occupied == true){
    ref = HIGH_LUX;
  } else {
    ref = LOW_LUX;
  }

  //Calculate the error between the current lux value and the refence value
  error=(ref-measureLux);
  
  //Proportinal Term calculation
  pterm = ref * kp * b - kp * measureLux;
  
  //Integral Term calculation
  iterm += ((error + lasterror)* ki * kp)*(T/2);
  
  //Derivative term calculation
  dterm = kd/(kd+a*T) * lastlux - kp*kd*a/(kd+a*T)*(measureLux-lastlux);
 
  //Calculate the new brightness value with the feedback in mind
  output = (pterm+iterm+dterm)- simulator();

  
  //Covert from lux to pwm
  output = brightness + luxToPWM(output);
  analogWrite(luminaire,output);
  

  //Save values for next iteration
  lastoutput = output;
  lastlux = measureLux;
  lasterror = error;

  //Return output value from 0 to 100
  return map(output,0,255,0,100);
}

//Converts from lux to pwm value. Calibrated for the situation in test
float luxToPWM(double lux){
  float pwm;
  pwm = lux * A_LUXTOPWM +B_LUXTOPWM;
  //Prevents overflow of the output values
  if(pwm > 255){
    pwm = 255;
  } else if(pwm < 0){
    pwm = 0;
  }
  return pwm;
}

//Simulates the delay caused by the charging of the capacitor
float simulator(){
  float desfasamento = 0;

  
  return desfasamento;
}



