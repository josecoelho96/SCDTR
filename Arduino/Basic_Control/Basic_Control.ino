//Arduino code for smart luminaire
//SDCTR 1S 2018/19
//Tecnico Lisboa
//David Teles-Jose Coelho-Afonso Soares
//ALL RIGHTS RESERVED


int luminaire = 9;
int sensor = A0;


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
  int brightness = calculateBrightness(lux);
  analogWrite(luminaire, brightness);
  Serial.print("Modulated Output");
  Serial.println(brightness);
}




int calculateBrightness(float lux){

  //implementar um controlador que altera o intenssidade do led conforme a iluminaçao


}

