const int sensor = A0;
const int luminaire = 3;
const int brightnessSteps = 13; // 0 .. 13
const int ledBrightnessLevels[] = {0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 255};
int currentBrightnessLevel = 0;
int levelInc = 1;

void setup() {
  Serial.begin(2000000);
  pinMode(luminaire, OUTPUT);
  currentBrightnessLevel = 0;
  levelInc = 1;

  // Configure Timers/Interrupts
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
  OCR1A = 15624; // T = 1s | f = 1 Hz
  
  // Configure ADC to run in free running mode
  // Resetting ADCSRA, ADCSRB and ADMUX registers
  // ADCSRA - ADC Control and Status Register A
  // ADCSRB - ADC Control and Status Register B
  // ADMUX - ADC Multiplexer Selection Register
  ADCSRA = B00000000;
  ADCSRB = B00000000;
  ADMUX = B00000000;

  // REFSn: Reference Selection [n = 1:0]
  // Set reference voltage to VCC : 01
  ADMUX |= (1 << REFS0); 

  /*
  // Setting prescaler (clk/32, f = 38.5 kHz because 13 cycles per conversion)
  // ADPS - ADC prescaler select
  ADCSRA |= (1 << ADPS2);
  ADCSRA |= (1 << ADPS0);
  */

  /*
  // Setting prescaler (clk/64, f = 19.2 kHz because 13 cycles per conversion)
  // ADPS - ADC prescaler select
  ADCSRA |= (1 << ADPS2);
  ADCSRA |= (1 << ADPS1);
  */

  // Setting prescaler (clk/128, f = 9.6 kHz because 13 cycles per conversion)
  // ADPS - ADC prescaler select
  ADCSRA |= (1 << ADPS2);
  ADCSRA |= (1 << ADPS1);
  ADCSRA |= (1 << ADPS0);
  
  // ADATE - ADC Auto Trigger Enable
  ADCSRA |= (1 << ADATE);
  // ADIE - ADC Interrupt Enable
  ADCSRA |= (1 << ADIE);
  // ADEN - ADC Enable
  ADCSRA |= (1 << ADEN);
  // ADSC - ADC Start Conversion
  ADCSRA |= (1 << ADSC);
  // Set to free running mode
  // ADTSn - ADC Auto Trigger Source [n = 2:0]
  ADCSRB &= ~(1 << ADTS2);
  ADCSRB &= ~(1 << ADTS1);
  ADCSRB &= ~(1 << ADTS0);

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

}

// Interrupt Service Routines
// Timer/Counter1 Compare Match A
ISR(TIMER1_COMPA_vect) {
  // Update LED Brightness (called 1 time per second)
 
  Serial.print("LED: ");
  Serial.print(micros());
  Serial.print("\t");
  Serial.println(ledBrightnessLevels[currentBrightnessLevel]);
  analogWrite(luminaire, ledBrightnessLevels[currentBrightnessLevel]);
  currentBrightnessLevel += levelInc;

  if (currentBrightnessLevel == brightnessSteps) {
    levelInc = -1;
  }
  if (currentBrightnessLevel == 0) {
    levelInc = 1;
  }
}

ISR(ADC_vect) {
  Serial.print(micros());
  Serial.print("\t");
  Serial.println(ADCL + (ADCH << 8));
}
