/*
  Title:    Davis Instruments 7911 Anemometer Test Code
  Date:     June 27, 2023
  Author:   Adam Garbo

  Notes:
  - INPUT_PULLUP used in place of an external resistor
  in deboucnce circuit to detect falling edges of wind
  speed interrupts

  Wiring Diagram
  ------------------------------
  Colour   Pin     Description
  ------------------------------
  Black    A1      Wind speed
  Green    A2      Wind direction
  Yellow   A4      Power (3.3V)
  Red      GND     Ground

*/

#define PIN_WIND_SPEED  A1
#define PIN_WIND_DIR    A2
#define PIN_SENSOR_PWR  A4  // 3.3V power

volatile int  revolutions       = 0;    // Wind speed ISR counter
float         windSpeed         = 0.0;  // Wind speed (m/s)
float         windDirection     = 0.0;  // Wind direction (°)
float         windGustSpeed     = 0.0;  // Wind gust speed  (m/s)
float         windGustDirection = 0.0;  // Wind gust direction (°)
int           sensorValue       = 0;

void setup()
{
  pinMode(PIN_SENSOR_PWR, OUTPUT);
  digitalWrite(PIN_SENSOR_PWR, LOW);

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

  Serial.println("revolutions,wind speed,analog,wind direction");
}

void loop()
{
  read7911();
  delay(1000);
}

void read7911()
{
  uint32_t loopStartTime = millis();

  // Enable pull-ups
  pinMode(PIN_WIND_SPEED, INPUT_PULLUP);

  // Attach interrupt to wind speed input pin
  attachInterrupt(PIN_WIND_SPEED, windSpeedIsr, FALLING);
  revolutions = 0;

  // Measure wind speed for 3 seconds
  while (millis() < loopStartTime + 3000);
  {
    // Do nothing
  }

  // Detach interrupt from wind speed input pin
  detachInterrupt(PIN_WIND_SPEED);

  // Disable pull-ups
  pinMode(PIN_WIND_SPEED, INPUT);

  // Calculate wind speed according to Davis Instruments formula: V = P(2.25/T)
  // V = speed in miles per hour
  // P = no. of pulses in sample period
  // T = duration of sample period in seconds
  windSpeed = revolutions * (2.25 / 3);   // Calculate wind speed in miles per hour
  windSpeed *= 0.44704;                   // Convert wind speed to metres per second

  // Enable power
  digitalWrite(PIN_SENSOR_PWR, HIGH);

  // Measure wind direction
  (void)analogRead(PIN_WIND_DIR);
  sensorValue = analogRead(PIN_WIND_DIR); // Raw analog wind direction value
  windDirection = map(sensorValue, 0, 4095, 0, 359); // Map wind direction to degrees (0-360°)

  // Disable power
  digitalWrite(PIN_SENSOR_PWR, LOW);

  /*
    // Correct for negative wind direction values
    if (windDirection > 360)
      windDirection -= 360;
    if (windDirection < 0)
      windDirection += 360;
  */
  // Print debug info

  Serial.print(revolutions); Serial.print(",");
  Serial.print(windSpeed); Serial.print(",");
  Serial.print(sensorValue); Serial.print(",");
  Serial.println(windDirection);

}

// Interrupt service routine (ISR) for wind speed measurement
// for Davis Instruments 7911 anemometer
void windSpeedIsr()
{
  revolutions++;
}
