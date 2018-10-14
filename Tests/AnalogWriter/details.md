# Details on AnalogWriter testing
All measurements were made using the `AnalogReaderV4` code.

## V1 - Basic code (no custom prescaling - clk/64)
**Results:**
- PWM freq: 488 Hz (Nominal 490.20 Hz)

## V2 - Prescaling to clk/128
**Results:**
- PWM freq: 243 Hz  [NOMINAL: 245.10]

## V3 - Prescaling to clk/32
**Results:**
- PWM freq: 980 Hz  [NOMINAL: 980.39]

## V4 - Prescaling to clk/8
**Results:**
- PWM freq: 3846 Hz  [NOMINAL: 3921.16]

## V5 - Prescaling to clk/1
**Results:**
- PWM freq: 15625 Hz  [NOMINAL: 31372.55]
