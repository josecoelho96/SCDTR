// Arduino code for smart luminaire
// SCDTR 1S 2018/19
// Tecnico Lisboa
// David Teles - Jose Coelho - Afonso Soares
// ALL RIGHTS RESERVED

// Config/parameters
// #define DEBUG
#define LOOP_INFO
#define CALIBRATE

#include <EEPROM.h>



#define LOW_LUX 50
#define HIGH_LUX 120
#define WINDUPMAX 500
#define DEADZONE 2
#define EEPROM_FIRST_ADD 0



const int Vref = 5; // ADC referece voltage
const int R1ref = 9850; // R1 measured value
const long int maxResistanceLdr = 1000000; //1MOhm, defined by datasheet
const float mLdr = -0.652; // LDR characteristic curve: m parameter
const float bLdr = 1.76; // LDR characteristic curve: b parameter



//Contants used by the PID controle
const double tau = 1;
const double a = 10;
const double b = 1;
const double T = 0.005; //2Khz Period
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

  noInterrupts();
  attachInterrupt(digitalPinToInterrupt(presenceSensor), stateChange, CHANGE);

  // Enable faster PWM on Pin 3 (and 11);
  // Reset TCCR2B register Clock Select (CS) bits
  // TC2 - Timer/Counter1 (8 bits)
  // TCCR2B - TC2 Control Register B
  TCCR2B &= B11111000;
  // Setting no prescaler: 001
  // TCCR2B |= (1 << CS22);
  // TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);


  //Setup Timer for feedback loop (500 Hz)
  // Resetting TCCR1A and TCCR1B registers.
  // TC1 - Timer/Counter1 (16 bits)
  // TCCR1A - TC1 Control Register A
  // TCCR1B - TC1 Control Register B
  TCCR1A = B00000000;
  TCCR1B = B00000000;

  // Changing mode of operation to CTC Mode (Clear Timer on Compare Match)
  // WGM - Waveform Generation Mode
  TCCR1B |= (1 << WGM12);

  // Adjusting prescaler (clk/1024, f = 15.625 kHz, T = 64 us)
  // CS - Clock Select
  TCCR1B |= (1 << CS12);
  TCCR1B |= (1 << CS10);

  //Setting interrupts to be called on counter match with OCR1A.
  // TIMSK1 - Timer/Counter 1 Interrupt Mask Register
  // OCIEA - Output Compare A Match Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  // Output compare register value.
  // Interruptions will occur at each (OCR1A+1)*T seconds. (T = 64 us)
  // T_interrupt = (OCR+1)*T
  // f_interrupt = 16MHz /((OCR+1)*1024)
  // OCR1A = 15624; // T = 1 s | f = 1 Hz
  // OCR1A = 31249; // T = 2 s | f = 0.5 Hz
  OCR1A = 31; // T = 2 ms | f = 488 Hz

  //Setting interrupts to be called on counter match with OCR1A.
  // TIMSK1 - Timer/Counter 1 Interrupt Mask Register
  // OCIEA - Output Compare A Match Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

  calibrate();
  feedForward();
}

void loop() {

  //#ifdef LOOP_INFO
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
  //#endif
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
  measuredLux = getLDRLux();
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
  error=(ref-measuredLux);

  if(error < abs(DEADZONE)){
    error = 0;
  }


  
  //Proportinal Term calculation
  pterm = ref * kp * b - kp * measuredLux;
  
  //Integral Term calculation
  iterm += ((error + lasterror)* ki * kp)*(T/2);


  //Anti WindUp
  if(iterm > WINDUPMAX){
    iterm = WINDUPMAX;
  } else if(iterm < -WINDUPMAX){
    iterm = -WINDUPMAX;
  }


  
  //Derivative term calculation
  dterm = kd/(kd+a*T) * lastlux - kp*kd*a/(kd+a*T)*(measuredLux-lastlux);
 
  //Calculate the new brightness value with the feedback in mind
  output = (pterm+iterm+dterm)- simulator();

  
  //Covert from lux to pwm
  output = brightness + luxToPWM(output);
  analogWrite(luminaire,output);
  

  //Save values for next iteration
  lastoutput = output;
  lastlux = measuredLux;
  lasterror = error;

  //Return output value from 0 to 100
  return map(output,0,255,0,100);
}

void calibrate(){
  float a, b, point1, point2;
  #ifdef CALIBRATE
    analogWrite(luminaire,0);
    delay(1000);
    b = getLDRLux();
    delay(50);
    analogWrite(luminaire,127);
    delay(1000);
    point1 = getLDRLux();
    delay(50);
    analogWrite(luminaire,254);
    delay(1000);
    point2 = getLDRLux();
    delay(50);
    analogWrite(luminaire,0);
    a=(254-127)/(point2-point1);
    EEPROM.put(EEPROM_FIRST_ADD,a);
    EEPROM.put(EEPROM_FIRST_ADD + sizeof(float),b);
    Serial.println("Calibration: ");
    Serial.print("a: ");
    Serial.println(a);
    Serial.print("b: ");
    Serial.println(b);
  #endif

}



//Converts from lux to pwm value. Calibrated for the situation in test
float luxToPWM(double lux){
  float pwm, a, b;
  EEPROM.get(EEPROM_FIRST_ADD , a);
  EEPROM.get(EEPROM_FIRST_ADD + sizeof(float) , b);
  pwm = lux * a + b;
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

void stateChange(){
  occupied = !digitalRead(presenceSensor);
  feedForward();
}

ISR(TIMER1_COMPA_vect) {
  feedBack();

}
