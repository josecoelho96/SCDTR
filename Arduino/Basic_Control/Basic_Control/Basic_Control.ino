// Arduino code for smart luminaire
// SCDTR 1S 2018/19
// Tecnico Lisboa
// David Teles - Jose Coelho - Afonso Soares
// ALL RIGHTS RESERVED

// ============================== LOGGING CONTROL =============================
// #define DEBUG
// #define LOOP_INFO
// #define FEEDBACK_DEBUG
// #define TIMING_DEBUG
#define PLOT_SYSTEM

// ============================== LOGGING CONTROL =============================

// ============================== Calibration =============================

#define STARTCALIBRATEBYTE 1
#define CALIBRATIONVALUEBYTE 2
byte k[254][2];
// ============================== Calibration =============================

// ============================== I2C CONTROL =============================

#include <Wire.h>
byte x = 0;
byte address = 1;

// ============================== I2C CONTROL =============================

// ================================== PINOUT ==================================
const int luminaire = 3;
const int presenceSensor = 2;
const int lightSensor = A0;
// ================================== PINOUT ==================================


// ============================== STATE VARIABLES =============================
volatile bool occupied; // True if presence detected, False otherwise
volatile int targetLux; // Changed according occupation and LOW_LUX or HIGH_LUX
volatile bool updateFeedback; // True if feedback parameters should be updated
// ============================== STATE VARIABLES =============================


// ============================= CONTROL VARIABLES ============================
// const float T = 0.002; // freq: 500 Hz
const float T = 0.004; // freq: 250 Hz
// const float T = 0.01; // freq: 100 Hz
// ============================= CONTROL VARIABLES ============================


// ============================= SYSTEM CONSTANTS =============================
const int Vref = 5; // ADC referece voltage
const int R1ref = 9850; // R1 measured value
const long int maxResistanceLdr = 1000000; //1MOhm, defined by datasheet
const float mLdr = -0.652; // LDR characteristic curve: m parameter
const float bLdr = 1.76; // LDR characteristic curve: b parameter
const float Kp = 1; // Proportional gain
const float Ki = 0.5; // Integral gain
// const float Kd = 0; // Derivative gain
#define WINDUPMAX 160 // Max value for integrator term
const float mLuxPWMConverter = 0.5519;
const float bLuxPWMConverter = 1.2202;
const int bPID = 1;
// ============================= SYSTEM CONSTANTS =============================


// ============================= SYSTEM PARAMETERS ============================
#define LOW_LUX 50
#define HIGH_LUX 100
#define DEADZONE 2
int controlSignal; // LED PWM
int controlSignalFeedforward;
float measuredLux;
float p;
float i;
float i_prev;
float error_prev;
// ============================= SYSTEM PARAMETERS ============================

void setup() {
// ============================== I2C CONTROL =============================

  Wire.begin(address);
  //Wire.onReceive(receiveEvent);
  TWAR = TWAR | B00000001;

// ============================== I2C CONTROL =============================

  // Stop all interrupts untill all registers are set
  noInterrupts();

  // ============================== TIMER CONTROL =============================
  // ==================== TIMER2 : Faster PWM on pin 3, 11 ====================
  // Reset TCCR2B register Clock Select (CS) bits
  // TC2 - Timer/Counter2 (8 bits)
  // TCCR2B - TC2 Control Register B
  TCCR2B &= B11111000; // Mask other bits
  // Setting no prescaler: 001
  // TCCR2B |= (1 << CS22);
  // TCCR2B |= (1 << CS21);
  TCCR2B |= (1 << CS20);
  // ==================== TIMER2 : Faster PWM on pin 3, 11 ====================

  // ========================= TIMER1 : Feedback loop =========================
  // Resetting TCCR1A and TCCR1B registers.
  // TC1 - Timer/Counter1 (16 bits)
  // TCCR1A - TC1 Control Register A
  // TCCR1B - TC1 Control Register B
  TCCR1A = B00000000;
  TCCR1B = B00000000;

  // Changing mode of operation to CTC Mode (Clear Timer on Compare Match)
  // WGM - Waveform Generation Mode
  TCCR1B |= (1 << WGM12);

  // CS - Clock Select
  // Prescalers
  // - 1024 : 101
  // - 256  : 100
  TCCR1B |= (1 << CS12);
  // TCCR1B |= (1 << CS11);
  // TCCR1B |= (1 << CS10);

  // Setting interrupts to be called on counter match with OCR1A.
  // TIMSK1 - Timer/Counter 1 Interrupt Mask Register
  // OCIEA - Output Compare A Match Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);

  // OCR1A: Output Compare Register
  // Interruptions will occur at each (OCR1A+1)*T seconds
  // T_interrupt = (OCR+1)*T
  // f_interrupt = 16MHz /((OCR+1)*prescaler) = f_scaled/(OCR+1)
  // For 1024 prescaler, T = 64 us, f = 15.625 kHz
  // For 256 prescaler, T = 16 us, f = 62.5 kHz <---

  // OCR1A = 124; // T = 2 ms | f = 500 Hz
  OCR1A = 249; // T = 4 ms | f = 250 Hz
  // OCR1A = 624; // T = 10 ms | f = 100 Hz

  // ========================= TIMER1 : Feedback loop =========================
  // ============================== TIMER CONTROL =============================

  // ==================================== IO ==================================
  pinMode(luminaire, OUTPUT);
  pinMode(lightSensor, INPUT);
  pinMode(presenceSensor, INPUT_PULLUP);
  // ==================================== IO ==================================

  // Attach the presence sensor to an interrupt
  attachInterrupt(digitalPinToInterrupt(presenceSensor), stateChange, CHANGE);

  // Check for initial state (occupied or not)
  stateChange();
  updateFeedback = false;
  p = 0;
  i = 0;
  measuredLux = 0;
  i_prev = 0;
  error_prev = 0;

  // Start Serial
  Serial.begin(2000000);

  // Setup is complete, re-enable interrupts
  interrupts();
}

void loop() {
  if (updateFeedback == true) {
    feedback();
    updateFeedback = false;
  }

  #ifdef LOOP_INFO
    Serial.print("Luminance [Lux]: ");
    Serial.println(measuredLux);
    Serial.print("Occupied [Boolean]: ");
    Serial.println(occupied);
    Serial.print("Brightness [PWM]: ");
    Serial.println(controlSignal);
    Serial.print("pTerm]: ");
    Serial.println(p);
    Serial.print("iTerm: ");
    Serial.println(i);
    delay(500);
  #endif

  #ifdef PLOT_SYSTEM
    Serial.print(micros());
    Serial.print("\t");
    Serial.print(controlSignal); // PWM value
    Serial.print("\t");
    Serial.println(measuredLux); // LDR measured
  #endif
}

void feedforward(){
  // Get corresponding PWM value to desired lux
  controlSignal = luxToPWM(targetLux);
  analogWrite(luminaire, controlSignal);
  controlSignalFeedforward = controlSignal;
}

void feedback() {
  #ifdef TIMING_DEBUG
    Serial.print("s:");
    Serial.println(micros());
  #endif

  float errorRaw; // Error before deadzone filtering
  float error; // Error after deadzone filtering
  float u_feedback;

  measuredLux = getLDRLux();
  #ifdef FEEDBACK_DEBUG
    Serial.print("Measured lux: ");
    Serial.println(measuredLux);
  #endif

  //Calculate the error between the current lux value and the refence value
  errorRaw = targetLux - measuredLux;

  // Apply deadzone filtering
  error = deadzone_filtering(errorRaw);
  #ifdef FEEDBACK_DEBUG
    Serial.print("Lux error: ");
    Serial.println(error);
  #endif

  // PI controller calculations
  p = Kp * bPID * targetLux - Kp * measuredLux;
  i = i_prev + (Kp * Ki * T/2) * (error + error_prev);

  // Block integrator term (basic integrator-windup solution)
  if (i > WINDUPMAX) {
    i = WINDUPMAX;
  } else if (i < -WINDUPMAX) {
    i = -WINDUPMAX;
  }

  // Compute control signal
  // u = p + i + d - simulator();

  u_feedback = p + i;
  #ifdef FEEDBACK_DEBUG
    Serial.print("U feedback: ");
    Serial.println(u_feedback);
  #endif

  controlSignal = controlSignalFeedforward + u_feedback;
  if (controlSignal > 255) {
    controlSignal = 255;
  } else if (controlSignal < 0) {
    controlSignal = 0;
  }

  #ifdef FEEDBACK_DEBUG
    Serial.print("control signal: ");
    Serial.println(controlSignal);
  #endif
  // Produce output
  analogWrite(luminaire, controlSignal);
  #ifdef TIMING_DEBUG
    Serial.print("u:");
    Serial.println(micros());
  #endif

  // Save values for next iteration
  i_prev = i;
  error_prev = error;

  #ifdef TIMING_DEBUG
    Serial.print("e:");
    Serial.println(micros());
  #endif
}

// Converts from lux to PWM value, based on lab experiments
int luxToPWM(float lux) {
  // lux = pwm * m + b
  // => PWM = (lux - b) / m
  int pwm;
  pwm = (lux - bLuxPWMConverter) / mLuxPWMConverter;
  //Prevents overflow of the output values
  if (pwm > 255) {
    pwm = 255;
  } else if (pwm < 0) {
    pwm = 0;
  }
  return pwm;
}

/*
//Simulates the delay caused by the charging of the capacitor
float simulator(){
  float desfasamento = 0;

  
  return desfasamento;
}
*/


// Converts an ADC reading (from a LDR) to lux
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
    // Complete darkness, use value from datasheet
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

// Deadzone filtering
float deadzone_filtering(float error) {
  // TODO: Which method?
  // ============================ METHOD 1 (Slides) ===========================
  // Symmetrical with band equal to zero. No gaps and slope = 1
  if (abs(error) <= DEADZONE) {
    return 0;
  } else if (error < DEADZONE) {
    return error + DEADZONE;
  } else if (error > DEADZONE) {
    return error - DEADZONE;
  }
  // ============================ METHOD 1 (Slides) ===========================


  // ========================== METHOD 2 (Wikipedia) ==========================
  // Symmetrical with band equal to zero. Gap and slope = 1
    // if (abs(error) <= DEADZONE) {
    //   return 0;
    // } else {
    //   return error;
    // }
  // ========================== METHOD 2 (Wikipedia) ==========================
}



//========================== I2C comunication ==========================
//Limites
//K
//Consensus
void sendConsensusI2C ( int limit , int consensus ){

  Wire.beginTransmission(0);
  Wire.write(address);
  Wire.write('l');
  Wire.write(limit);
  Wire.write('c');
  Wire.write(limit);
  Wire.endTransmission();

}

void startCalibration(){

  //Set led to max output
  analogWrite(luminaire,255);
  //Send calibration command
  Wire.beginTransmission(0);
  Wire.write(address);
  Wire.write(STARTCALIBRATEBYTE);
  Wire.write(0);
  Wire.endTransmission();
  //Wait for all arduinos to receive and respond
  
  while(Wire.available()){
    byte ad = Wire.read();
    byte type = Wire.read();
    if(type == CALIBRATIONVALUEBYTE){
      byte msgsize = Wire.read();
      for(int i = 0; i <= msgsize; i++){
            byte rx = Wire.read();
            k[ad][0] = rx;
      }
    }
    

  }
  

  
  
  
}


void readViaI2C(){

  while (Wire.available()) {
    byte rx = Wire.read();
    Serial.println(rx);
  } 

  
}




//========================== I2C comunication ==========================


void stateChange() {
  // Update occupied state
  occupied = !digitalRead(presenceSensor);

  // Update target lux value
  if (occupied == true) {
    targetLux = HIGH_LUX;
  } else {
    targetLux = LOW_LUX;
  }

  // Update system directly
  feedforward();
}

ISR(TIMER1_COMPA_vect) {
  // Activate flag
  updateFeedback = true;
}
