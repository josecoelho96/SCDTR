//Arduino code for smart luminaire
//SCDTR 1S 2018/19
//Tecnico Lisboa
//David Teles-Jose Coelho-Afonso Soares
//ALL RIGHTS RESERVED - LMAO

#include <math.h> 

int luminaire = 9;
int button = 8;//NEW
int sensor = A0;
int brightness;
float lastLux;
int lowLux=200;
int highLux=500;
float sensorVoltage = 0;
// int counter = 0;
// float acc_value = 0;
float sensorResistance = 0;
float luminance = 0;

void setup() {
  Serial.begin(9600);
  pinMode(luminaire, OUTPUT);
  pinMode(button, OUTPUT);//NEW
  digitalWrite(button,HIGH);//NEW
  pinMode(sensor, INPUT);
  // counter = 0;
  // acc_value = 0;
}

void loop() {
  int sensorValue = analogRead(sensor);
  Serial.print("LDR [ADC value]: ");
  Serial.println(sensorValue);
  sensorVoltage = (5*sensorValue)/1023.0;
  Serial.print("LDR [Voltage at terminal]: ");
  Serial.print(sensorVoltage);
  Serial.println(" V");
  sensorResistance = (5/sensorVoltage)*10000 - 10000;
  Serial.print("LDR [Resistance]: ");
  Serial.print(sensorResistance);
  Serial.println(" Ohm");
  float b = 1.774073098;
  float m = -0.6570013422;
  luminance = pow(10, (log10(sensorResistance/1000)-b)/m);
  Serial.print("LDR [Luminance]: ");
  Serial.print(luminance);
  Serial.println(" lux");
  calculateBrightness(luminance, digitalRead(button));
   Serial.print("State [Ocupied]: ");
   Serial.println(digitalRead(button));
  delay(1000);
}

int calculateLux(int ldrReading) {
  /* Values according to datasheet
   * 0 lux: 1 MOhm
   * 1 lux : 60 kOhm
   * 10 lux : 15 kOhm
   * 100 lux : 3 kOhm
   */
  
  // float lux = map(sensorValue, 200, 1016, 0, 100); //Valores nao confirmados pelo datasheet nem calibrados
  // return lux;
  
}

int calculateBrightness(float lux, bool ocupation){
  int output;
  int luxError;
  if (ocupation == true){
    luxError=abs(lux-highLux);
  } else {
    luxError=abs(lux-lowLux);
  }
  if (brightness!=0){
     output = brightness * (1+(luxError/100));

  } else {
    output = 10 * (1+(luxError/100));
  }

  //implementar um controlador que altera o intenssidade do led conforme a iluminaçao
  return output;
}

