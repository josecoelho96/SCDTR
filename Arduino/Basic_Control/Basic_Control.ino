//Arduino code for smart luminaire
//SDCTR 1S 2018/19
//Tecnico Lisboa
//David Teles-Jose Coelho-Afonso Soares
//ALL RIGHTS RESERVED


int luminaire = 9;
int sensor = A0;
int brightness;
float lastLux;
int lowLux=200;
int highLux=500;


void setup() {
  Serial.begin(9600);
  pinMode(luminaire, OUTPUT);
  pinMode(sensor, INPUT);
  

}

void loop() {
  int sensorValue = analogRead(sensor);
  Serial.print("LDR Output");
  Serial.println(sensorValue);
  float lux = map(sensorValue, 0, 1023, 0, 100);//Valores nao confirmados pelo datasheet nem calibrados
  Serial.print("Lux:");
  Serial.println(lux);
  brightness = calculateBrightness(lux, true);
  analogWrite(luminaire, map(brightness, 0, 100, 0, 255));
  Serial.print("Modulated Output");
  Serial.println(brightness);
  delay(50);
}




int calculateBrightness(float lux, bool ocupation){
  int output;
  int luxError;
  if(ocupation == true){
    luxError=abs(lux-highLux);
    

    
  } else {
    luxError=abs(lux-lowLux);
    


  
  }

    if(brightness!=0){
       output = brightness * (1+(luxError/100));

    } else {
      output = 10 * (1+(luxError/100));
    }

  //implementar um controlador que altera o intenssidade do led conforme a iluminaçao
  return output;

}

