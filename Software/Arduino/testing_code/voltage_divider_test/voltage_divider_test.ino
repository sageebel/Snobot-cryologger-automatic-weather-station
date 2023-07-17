/*
  Title:    10/1 MOhm Resistor Voltage Divider Test Code
  Date:     June 29, 2023
  Author:   Adam Garbo

  Notes:
  - Code to test battery voltage measured across
  10/1 MOhm resistor divider
*/

#define PIN_VBAT    A0
#define PIN_12V_EN  5   // 12 V step-up/down regulator
#define PIN_5V_EN   6   // 5V step-down regulator

void setup()
{
  pinMode(PIN_5V_EN, OUTPUT);
  pinMode(PIN_12V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, LOW);   // Disable 5V power
  digitalWrite(PIN_12V_EN, LOW);  // Disable 12V power

  Serial.begin(115200);
  while (!Serial);

  // Configure ADC
  ADC->CTRLA.bit.ENABLE = 0;                      // Disable ADC
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |   // Divide Clock ADC GCLK by 512 (48MHz/512 = 93.7kHz)
                   ADC_CTRLB_RESSEL_16BIT;        // Set ADC resolution to 12-bit
  while (ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization
  ADC->SAMPCTRL.reg = ADC_SAMPCTRL_SAMPLEN(64);   // Set Sampling Time Length (341.33 us)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_512 |  // Configure multisampling
                     ADC_AVGCTRL_ADJRES(4);       // Configure averaging
  while (ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization
  ADC->CTRLA.bit.ENABLE = 1;                      // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization

  // Apply ADC gain and offset error calibration correction
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(0);
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(2048);
  ADC->CTRLB.bit.CORREN = true;
  while (ADC->STATUS.bit.SYNCBUSY);               // Wait for synchronization

}

void loop()
{
  float sensorValue = analogRead(PIN_VBAT);
  float voltage1 = sensorValue * (3.3 / 4096.0);
  float voltage2 = sensorValue * ((10000000.0 + 1000000.0) / 1000000.0); // Multiply back 1 MOhm / (10 MOhm + 1 MOhm)
  voltage2 *= 3.3;   // Multiply by 3.3V reference voltage
  voltage2 /= 4096;  // Convert to voltage
  Serial.print(F("sensorValue: ")); Serial.print(sensorValue); Serial.print(F(",")); Serial.print(voltage1, 4); Serial.print(F(",")); Serial.println(voltage2, 4);

  delay(500);
}
