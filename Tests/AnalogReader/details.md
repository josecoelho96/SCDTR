# Details on AnalogReader testing

## V1 - Basic code
Outputs samples as soons as data is read.

**Results:**
- Avg 540 us per sample -> 1852 S/s
- PWM freq: 581 Hz (Nominal 490.20 Hz)

## V2 - Basic code with memory
Outputs samples into memory. Memory is dumped into Serial when full.

**Results:**
- Avg 112 us per sample -> 8928 S/s (Between successive readings)
- PWM freq: 496 Hz (Nominal 490.20 Hz)

## V3 - Increased sample frequency
Outputs samples as soons as data is read.

**Results:**
- Avg 467 us per sample -> 2140 S/s
- PWM freq: 538 Hz (Nominal 490.20 Hz)


## V4 - Increased sample frequency with memory
Outputs samples into memory. Memory is dumped into Serial when full.

**Results:**
- Avg 467 us per sample -> 50 kS/s
- PWM freq: 488 Hz (Nominal 490.20 Hz)

## V5 - Increased sample frequency with memory and interrupts
Outputs samples into memory. Memory is dumped into Serial when full. Using ADC in free running mode.

**Results:**
- Avg 16 us per sample -> 62.5 kS/s
- PWM freq: 486 Hz (Nominal 490.20 Hz)

