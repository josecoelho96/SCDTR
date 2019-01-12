#include "MessageTypes.h"
#include "Wire.h"

// Arduino code for smart luminaire
// SCDTR 1S 2018/19
// Tecnico Lisboa
// David Teles - Jose Coelho - Afonso Soares
// ALL RIGHTS RESERVED

// ============================= CONTROL VARIABLES ============================
const float T = 0.01; // freq: 100 Hz
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
#define WINDUPMAX 80 // Max value for integrator term
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
unsigned long lastFeedforwardChange;
float Vf;
float Vi;
float measuredLux;
float p;
float i;
float i_prev;
float error_prev;
// ============================= SYSTEM PARAMETERS ============================


// ============================== STATE VARIABLES =============================
volatile bool occupied; // True if presence detected, False otherwise
volatile int targetLux; // Changed according occupation and LOW_LUX or HIGH_LUX
volatile bool updateFeedback; // True if feedback parameters should be updated
// ============================== STATE VARIABLES =============================

// ================================== PINOUT ==================================
const int luminaire = 3;
const int presenceSensor = 2;
const int lightSensor = A0;
// ================================== PINOUT ==================================

// ============================== GLOBAL SETTINGS =============================
const int MAX_NODES = 3; // Defined the max number of nodes (neighbours)
const int TIMEOUT = 2000; // ms
const int CALIBRATION_LED_ON_TIME_MIN_TIME = 1000; //ms
const int CALIBRATION_LED_ON_TIME = 3000; //ms
const int CALIBRATION_LDR_READS = 100;
// ============================== GLOBAL SETTINGS =============================

// =============================== NODE SETTINGS ==============================
const byte address = 15;
// =============================== NODE SETTINGS ==============================

volatile int current_neighbour_nodes = 0;
volatile byte neighbour_nodes_addresses[MAX_NODES];
volatile float consensus_K[2]; // TODO: Only working for 2 nodes!

// =================================== FLAGS ==================================
volatile boolean f_send_network_full = false;
volatile boolean f_node_already_on_network = false;
volatile boolean f_send_joined_network = false;
volatile boolean f_joined_network = false;
volatile boolean f_in_network = false;

volatile byte last_node_led_on = 0;
volatile boolean f_calibration_mode = false;
volatile boolean f_calibration_next_light = false;
volatile boolean f_calibration_led_on = false;
volatile boolean f_calibration_need_to_light_led_on = false;
volatile boolean f_calibration_measure_ldr = false;
unsigned long start_time_calibration_led_on_min_time;
unsigned long start_time_calibration_led_on;

volatile boolean f_setup = true;


// ================================ CALIBRATION ===============================
volatile float k = 0;
volatile int number_of_readings = 0;
// ================================ CALIBRATION ===============================


// =================================== FLAGS ==================================


void setup() {

  // Stop all interrupts untill all registers are set
  noInterrupts();

  // Debug only
  Serial.begin(9600);

  // Reset all values to defaults
  current_neighbour_nodes = 0;
  for (int i = 0; i < MAX_NODES; i++) {
    neighbour_nodes_addresses[i] = 0;
  }
  f_send_network_full = false;
  f_node_already_on_network = false;
  f_send_joined_network = false;
  f_joined_network = false;
  f_in_network = false;
  last_node_led_on = 0;
  f_calibration_mode = false;
  f_calibration_next_light = false;
  f_calibration_led_on = false;
  f_calibration_need_to_light_led_on = false;
  f_calibration_measure_ldr = false;
  k = 0;
  number_of_readings = 0;

  f_setup = true;


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
  OCR1A = 624; // T = 10 ms | f = 100 Hz

  // ========================= TIMER1 : Feedback loop =========================
  // ============================== TIMER CONTROL =============================

  // ==================================== IO ==================================
  pinMode(luminaire, OUTPUT);
  pinMode(lightSensor, INPUT);
  pinMode(presenceSensor, INPUT_PULLUP);
  // ==================================== IO ==================================

  // Setup is complete, re-enable interrupts
  interrupts();

  analogWrite(luminaire, 0);

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

  startI2C();
  if (requestJoinNetwork()) {
    requestForCalibration();
  }

  f_setup = false;
}

void loop() {
  // Send messages as needed
  if (f_send_network_full) {
    sendData(MT_MAX_NODES_IN_NETWORK_REACHED);
    f_send_network_full = false;
  }

  if (f_send_joined_network) {
    sendData(MT_REQUEST_JOIN_NETWORK_REPLY_OK);
    f_send_joined_network = false;
  }

  if (f_calibration_mode) {
    calibrate();
  }

  if (updateFeedback == true && f_calibration_mode == false) {
    feedback();
    updateFeedback = false;
  }

  if (f_calibration_measure_ldr == true) {
    if (millis() - start_time_calibration_led_on_min_time > CALIBRATION_LED_ON_TIME_MIN_TIME) {
      // Add time to get stable
      if (number_of_readings < CALIBRATION_LDR_READS) {
        k += getLDRLux();
        // sendValueFloat(MT_CALIBRATION_VALUE, k);
        number_of_readings++;
      } else {
        sendValueFloat(MT_CALIBRATION_VALUE, k/CALIBRATION_LDR_READS);
        Serial.print("M");
        Serial.println(k/CALIBRATION_LDR_READS);


        f_calibration_measure_ldr = false;
      }
    }
  }
}

void calibrate() {

  if (last_node_led_on == 0) {
    // First one is the one with the lower address.
    // check if all my neighbours have higher addresses than me
    f_calibration_next_light = true; // assume I'm the node with the lowest address
    for (int i = 0; i < MAX_NODES; i++) {
      if (neighbour_nodes_addresses[i] != 0) { // Exclude 0's as they are not valid addresses
        if (neighbour_nodes_addresses[i] < address) { // A neighbour has a lower ID. Stop search
          // Some other node will find itself as starting the calibration procedure
          f_calibration_next_light = false;
          break;
        }
      }
    }
  } else {
    // Some LEDs have already been up, check if I should be the next one.
    f_calibration_next_light = true; // assume I'm the next node
    for (int i = 0; i < MAX_NODES; i++) {
      if (neighbour_nodes_addresses[i] != 0) {
        // Ignore all neighbours with lower addresses than the last one who sent a LED OFF
        if (last_node_led_on >= neighbour_nodes_addresses[i]) {
          continue;
        }
        // Other node as a lower but valid value
        if (neighbour_nodes_addresses[i] < address) {
          f_calibration_next_light = false;
          break;
        }
      }
    }
  }

  // If I'm the next node to light up my LED...
  if (f_calibration_next_light == true) {
    if (f_calibration_led_on == false && f_calibration_need_to_light_led_on == true) {
      start_time_calibration_led_on = millis();
      sendData(MT_CALIBRATION_LED_ON);
      last_node_led_on = address;
      analogWrite(3, 255);
      f_calibration_led_on = true;
      f_calibration_need_to_light_led_on = false;
    }

    if ((millis() - start_time_calibration_led_on > CALIBRATION_LED_ON_TIME) && f_calibration_led_on == true) {
      // Turn off LED
      analogWrite(3, 0);
      sendData(MT_CALIBRATION_LED_OFF);
      f_calibration_led_on = false;
      f_calibration_next_light = false;
    }
  }

  // If I have the highest of all addresses and don't need to light my led on and have my led off, send a end calibration message
  boolean last_node = true;
  for (int i = 0; i < MAX_NODES; i++) {
    if (neighbour_nodes_addresses[i] != 0) {
      // Ignore all neighbours with lower addresses than the last one who sent a LED OFF
      if (neighbour_nodes_addresses[i] > address) {
        last_node = false;
        break;
      }
    }
  }

  if (last_node == true && f_calibration_led_on == false && f_calibration_need_to_light_led_on == false) {
    // End calibration
    sendData(MT_END_CALIBRATION);
    f_calibration_mode = false;
    last_node_led_on = 0;
    f_calibration_mode = false;
    f_calibration_next_light = false;
    f_calibration_led_on = false;
    f_calibration_need_to_light_led_on = false;
  }
}

void startI2C() {
  Serial.print("Start I2C on address: ");
  Serial.println(address);
  Wire.begin(address);
  Wire.onReceive(receiveData);
  TWAR = TWAR | B00000001;
}

void receiveData(int howMany) {
  // TODO: ...
  // Serial.println("Recv data");
  byte header[3];
  int header_idx = 0;
  byte msg_float[4];
  float msg_value_f;
  int msg_idx = 0;

  while (header_idx < 3) {
    header[header_idx++] = Wire.read();
  }

  if (header[1] == MT_REQUEST_JOIN_NETWORK) {
    if (current_neighbour_nodes < MAX_NODES) {
      // check if not already on list
      f_node_already_on_network = false;
      for (int i = 0; i < current_neighbour_nodes; i++) {
        if (neighbour_nodes_addresses[i] == header[0]) {
          // node already on list, just don't add it again to the list
          f_node_already_on_network = true;
          break;
        }
      }
      if (f_node_already_on_network == false) {
        neighbour_nodes_addresses[current_neighbour_nodes++] = header[0];
      }
      f_send_joined_network = true;
      f_in_network = true;
      return;
    } else {
      f_send_network_full = true;
      return;
    }
  }

  if (header[1] == MT_REQUEST_JOIN_NETWORK_REPLY_OK) {
    // On production it should reply if it couldnt save all neighbours
    if (current_neighbour_nodes < MAX_NODES) {
      neighbour_nodes_addresses[current_neighbour_nodes++] = header[0];
      f_joined_network = true;
      f_in_network = true;
    } else {
      f_joined_network = false;
    }
    return;
  }

  if (header[1] == MT_REQUEST_FOR_CALIBRATION) {
    f_calibration_mode = true;
    f_calibration_need_to_light_led_on = true;
  }

  if (header[1] == MT_CALIBRATION_LED_ON) {
    //TODO: Anything?
    f_calibration_measure_ldr = true;
    number_of_readings = 0;
    start_time_calibration_led_on_min_time = millis();
    k = 0;
  }

  if (header[1] == MT_CALIBRATION_LED_OFF) {
    last_node_led_on = header[0];
    f_calibration_measure_ldr = false;
  }

  if (header[1] == MT_END_CALIBRATION) {
    f_calibration_mode = false;
    last_node_led_on = 0;
    f_calibration_mode = false;
    f_calibration_next_light = false;
    f_calibration_led_on = false;
    f_calibration_need_to_light_led_on = false;
  }

  if (header[1] == MT_CALIBRATION_VALUE) {
    // If on production, source address should be checked
    // Get the next 4 bytes - Getting the data from the other node.
    while (Wire.available()) {
      msg_float[msg_idx++] = Wire.read();
    }

    Serial.print(msg_float[0]);
    Serial.print("--");
    Serial.print(msg_float[1]);
    Serial.print("--");
    Serial.print(msg_float[2]);
    Serial.print("--");
    Serial.print(msg_float[3]);
    Serial.print("--");

    *((int*)(&msg_value_f) + 3) = msg_float[3];
    *((int*)(&msg_value_f) + 2) = msg_float[2];
    *((int*)(&msg_value_f) + 1) = msg_float[1];
    *((int*)(&msg_value_f) + 0) = msg_float[0];

  Serial.print("R");
  Serial.println(msg_value_f);
  consensus_K[1] =  msg_value_f;
  }
}

void sendData(byte type) {
  Wire.beginTransmission(0);
  Wire.write(address);
  Wire.write(type);
  Wire.endTransmission();
}

void sendValueFloat(byte type, float value) {
  Wire.beginTransmission(0);
  Wire.write(address);
  Wire.write(type);
  Wire.write(4);
  byte* data = (byte*)&value;
  Wire.write(data, 4);
  Wire.endTransmission();
}


// Returns False if no network found or network is full
boolean requestJoinNetwork() {
  // Serial.println("requestJoinNetwork");
  sendData(MT_REQUEST_JOIN_NETWORK);
  unsigned long start_time = millis();
  // Block for 2 secons (interrupts will be called)
  while (millis() - start_time < TIMEOUT) {}
  return f_joined_network;
}


void requestForCalibration() {
  // Serial.println("requestForCalibration");
  f_calibration_mode = true;
  f_calibration_need_to_light_led_on = true;
  last_node_led_on = 0;
  f_calibration_next_light = false;
  f_calibration_led_on = false;

  sendData(MT_REQUEST_FOR_CALIBRATION);
}

// Converts an ADC reading (from a LDR) to lux
float getLDRLux() {
  int adcLdr;
  float voltageLdr;
  float resistanceLdr;
  float luxLdr;

  adcLdr = analogRead(lightSensor);

  voltageLdr = (Vref*adcLdr)/1023.0;

  if (voltageLdr == 0) {
    // Complete darkness, use value from datasheet
    resistanceLdr = maxResistanceLdr;
  } else {
    resistanceLdr = R1ref*(Vref/voltageLdr - 1);
  }

  // Resistance is converted to kOhm
  luxLdr = pow(10, (log10(resistanceLdr/1000) - bLdr)/mLdr);

  return luxLdr;
}

void feedback() {

  float realTargetLux = targetLux;
  // realTargetLux = simulator();

  float errorRaw; // Error before deadzone filtering
  float error; // Error after deadzone filtering
  float u_feedback;

  measuredLux = getLDRLux();

  //Calculate the error between the current lux value and the refence value
  errorRaw = realTargetLux - measuredLux;

  // Apply deadzone filtering
  error = deadzone_filtering(errorRaw);

  // PI controller calculations
  p = Kp * bPID * realTargetLux - Kp * measuredLux;
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

  controlSignal = controlSignalFeedforward + u_feedback;
  if (controlSignal > 255) {
    controlSignal = 255;
  } else if (controlSignal < 0) {
    controlSignal = 0;
  }

  // Produce output
  analogWrite(luminaire, controlSignal);

  // Save values for next iteration
  i_prev = i;
  error_prev = error;
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
}

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
  if (f_calibration_mode == false && f_setup == false) {
    feedforward();
  }

}

void feedforward(){
  int adcLdr= analogRead(lightSensor);
  float R;
  Vi = (Vref*adcLdr)/1023.0;

  // Get corresponding PWM value to desired lux
  controlSignal = luxToPWM(targetLux);

  // R in ohm
  R = pow(10, mLdr*log10(targetLux) + bLdr);
  Vf = Vref/((R/R1ref) + 1);

  // Get corresponding PWM value to desired lux
  controlSignal = luxToPWM(targetLux);

  lastFeedforwardChange = millis();
  analogWrite(luminaire, controlSignal);
  controlSignalFeedforward = controlSignal;
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

// Simulates the delay caused by the charging of the capacitor
// Outputs actually expected lux value
float simulator() {
  unsigned long current_time = millis();
  float resistanceLdr;
  float v_adc;

  v_adc = Vf - (Vf - Vi)*exp((-current_time-lastFeedforwardChange)/time_constant());
  #ifdef DEBUG_SIMULATOR
    Serial.print("Vf: ");
    Serial.println(Vf);

    Serial.print("Vi: ");
    Serial.println(Vi);

    Serial.print("V_adc: ");
    Serial.println(v_adc);
  #endif
  if (v_adc == 0) {
    // Complete darkness, use value from datasheet
    resistanceLdr = maxResistanceLdr;
  } else {
    resistanceLdr = R1ref*(Vref/v_adc - 1);
  }

  // Returns real expected lux value
  return pow(10, (log10(resistanceLdr/1000) - bLdr)/mLdr);
}


// Outputs tau (the time constant) as a second order approximation
float time_constant() {
  float a = 0.008391063263791;
  float b = 0.029646566281119;
  float c = 0.021355097710248;
  return a * pow(2, Vf - Vi) + b * (Vf - Vi) + c;
}

ISR(TIMER1_COMPA_vect) {
  // Activate flag
  updateFeedback = true;
}
