//Arduino code for smart luminaire
//SCDTR 1S 2018/19
//Tecnico Lisboa
//David Teles-Jose Coelho-Afonso Soares
//ALL RIGHTS RESERVED


int luminaire = 9;
int sensor = A0;
int brightness;
float lastLux;
int lowLux=200;
int highLux=500;
int counter = 0;
float acc_value = 0;


void setup() {
  Serial.begin(9600);
  pinMode(luminaire, OUTPUT);
  pinMode(sensor, INPUT);
  counter = 0;
  acc_value = 0;
}

void loop() {
  int sensorValue = analogRead(sensor);
  Serial.print("LDR raw output: ");
  Serial.println(sensorValue);
  acc_value += sensorValue;
  counter++;
  Serial.print("LDR raw avg output: ");
  Serial.println(acc_value / counter);

  // TELES
  // 833 (adc) - 229 lux
  // 880 (adc) - 261 lux
  // 940 (adc) - 305 lux
  // 383 (adc) - X


  // COELHADAS
  // 360 (adc) - 5 lux
  // 830 (adc) - 264 lux
  // 890 (adc) - 1320 lux

/*
  Serial.print("Lux:");
  Serial.println(lux);
  brightness = lux; //100;//calculateBrightness(lux, true);
  analogWrite(luminaire, map(brightness, 0, 100, 255, 0));
  Serial.print("Modulated Output: ");
  Serial.println(brightness);
 */
  delay(500);
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

