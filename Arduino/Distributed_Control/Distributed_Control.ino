#include "MessageTypes.h"
#include "Wire.h"

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

volatile float k = 0;
volatile int number_of_readings = 0;
// ============================== Calibration =============================

// ============================== Consensus =============================

#define CONSENSUSBYTE 3
#define TARGETBYTE 4
int consensus;

// ============================== Consensus =============================
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

// ============================== GLOBAL SETTINGS =============================
const int MAX_NODES = 3; // Defined the max number of nodes
const int TIMEOUT = 2000; // ms
const int CALIBRATION_LED_ON_TIME = 2000; //ms
// ============================== GLOBAL SETTINGS =============================


// =============================== NODE SETTINGS ==============================
const byte address = 10;
// =============================== NODE SETTINGS ==============================

// =================================== FLAGS ==================================
volatile int current_neighbour_nodes = 0;
volatile byte neighbour_nodes_addresses[MAX_NODES];

volatile boolean f_send_network_full = false;
volatile boolean f_node_already_on_network = false;
volatile boolean f_send_joined_network = false;
volatile boolean f_joined_network = false;
volatile boolean f_in_network = false;
volatile boolean f_calibration_measure_ldr = false;

volatile byte last_node_led_on = 0;
volatile boolean f_calibration_mode = false;
volatile boolean f_calibration_next_light = false;
volatile boolean f_calibration_led_on = false;
volatile boolean f_calibration_need_to_light_led_on = false;

unsigned long start_time_calibration_led_on;


// =================================== FLAGS ==================================



void setup() {

  // Debug only
  Serial.begin(9600);

  pinMode(3, OUTPUT);
  pinMode(A0, INPUT);
  
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


  startI2C();
  if (requestJoinNetwork()) {
    requestForCalibration();
  }
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

  if(f_calibration_measure_ldr=true){
    k = getLDRLux();
    number_of_readings++;
  } else if(k != 0 && number_of_readings != 0){
    byte reading[4];
    k=k/number_of_readings;
    number_of_readings = 0;
    Wire.beginTransmission(0);
    Wire.write(address);
    Wire.write(MT_CALIBRATION_VALUE);
    Wire.write(4);
    *((float *) reading) = k;
    Wire.write(reading,4);
    Wire.endTransmission(); 
  }

  
  
    
  
  // ================ DEBUG ===================================
  // Serial.print("L");

  /*
  Serial.print(current_neighbour_nodes);
  Serial.print("-");
  for (int i = 0; i < MAX_NODES; i++) {
    Serial.print(neighbour_nodes_addresses[i]);
    if (MAX_NODES - i > 1) {
      Serial.print(":");
    }
  }
  */
  // Serial.print("-");
  // Serial.print(f_in_network);
  // Serial.print("-");
  // Serial.print(f_calibration_mode);

  // Serial.println("|");   
  // delay(500);
  // ================ DEBUG ==================================
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
  while (Wire.available()) {
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
}

void sendData(byte type) {
  Wire.beginTransmission(0);
  Wire.write(address);
  Wire.write(type);
  Wire.write(0); // TODO: Size!
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
  // unsigned long start_time = millis();
  // Block for 2 secons (interrupts will be called)
  // while (millis() - start_time < TIMEOUT) {}
}

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


