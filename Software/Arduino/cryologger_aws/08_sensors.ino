// ----------------------------------------------------------------------------
// Adafruit BME280 Temperature Humidity Pressure Sensor
// https://www.adafruit.com/product/2652
// ----------------------------------------------------------------------------
void configureBme280()
{
  DEBUG_PRINT("Info - Initializing BME280...");

  if (bme280.begin())
  {
    online.bme280 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    online.bme280 = false;
    DEBUG_PRINTLN("failed!");
  }
}

// Read BME280
void readBme280()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  // Initialize sensor
  configureBme280();

  // Check if sensor initialized successfully
  if (online.bme280)
  {
    DEBUG_PRINT("Info - Reading BME280...");

    myDelay(250);

    // Read sensor data
    temperatureInt = bme280.readTemperature();
    humidityInt = bme280.readHumidity();
    pressureInt = bme280.readPressure() / 100.0F;

    // Add to statistics object
    temperatureIntStats.add(temperatureInt);
    humidityIntStats.add(humidityInt);
    pressureIntStats.add(pressureInt);

    DEBUG_PRINTLN("done.");
  }
  else
  {
    DEBUG_PRINTLN("Warning - BME280 offline!");
  }
  // Stop the loop timer
  timer.readBme280 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Davis Instruments Temperature Humidity Sensor (Sensiron SHT31-LSS)
// ------------------------------
// Colour     Pin     Description
// ------------------------------
// Yellow    3.3V     Power
// Green     GND      Ground
// White     SCK      Clock
// Blue      SDA      Data
// ----------------------------------------------------------------------------
 void readSht31()
{
  // Start the loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINT("Info - Reading SHT31...");

  // Disable I2C bus
  Wire.end();

  // Add delay
  myDelay(100);

  // Read sensor
  temperatureExt = sht.readTemperatureC();
  humidityExt = sht.readHumidity();

  // Add to statistics object
  temperatureExtStats.add(temperatureExt);
  humidityExtStats.add(humidityExt);

  // Print debug info
  DEBUG_PRINT("Temperature: "); DEBUG_PRINT(temperatureExt); DEBUG_PRINTLN(" C");
  DEBUG_PRINT("Humidity: "); DEBUG_PRINT(humidityExt); DEBUG_PRINTLN("%");

  DEBUG_PRINTLN("done.");

  // Re-enable I2C bus
  Wire.begin();

  // Stop the loop timer
  timer.readSht31 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
// Adafruit LSM303AGR Accelerometer/Magnetomter
// https://www.adafruit.com/product/4413
// ----------------------------------------------------------------------------
void configureLsm303()
{
  DEBUG_PRINT("Info - Initializing LSM303...");


  // Initialize LSM303 accelerometer
  if (lsm303.begin())
  {
    online.lsm303 = true;
    DEBUG_PRINTLN("success!");
  }
  else
  {
    online.lsm303 = false;
    DEBUG_PRINTLN("failed!");
  }
}

void readLsm303()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Initialize accelerometer
  configureLsm303();

  // Check if sensor initialized successfully
  if (online.lsm303)
  {
    DEBUG_PRINT("Info - Reading LSM303...");

    myDelay(500);

    float xAvg = 0, yAvg = 0, zAvg = 0;

    // Read accelerometer data
    sensors_event_t accel;

    // Average accelerometer values
    int samplesToAverage = 30;
    for (int i = 0; i < samplesToAverage; ++i)
    {
      lsm303.getEvent(&accel);   // Read accelerometer data
      xAvg += accel.acceleration.x;
      yAvg += accel.acceleration.y;
      zAvg += accel.acceleration.z;
      delay(1);
    }
    // Calculate average
    xAvg /= samplesToAverage;
    yAvg /= samplesToAverage;
    zAvg /= samplesToAverage;

    // Calculate pitch and roll
    // Note: X-axis and Z axis swapped due to orientation of sensor when installed

    // Standard orientation (e.g., Igloolik)
    //pitch = atan2(-zAvg, sqrt(yAvg * yAvg + xAvg * xAvg)) * 180 / PI;
    //roll = atan2(yAvg, xAvg) * 180 / PI;

    // Rotated 90° orientation (e.g., Purple Valley)
    pitch = atan2(-zAvg, sqrt(yAvg * yAvg + xAvg * xAvg)) * 180 / PI;
    roll = atan2(yAvg, xAvg) * 180 / PI;

    // Write data to union
    moSbdMessage.pitch = pitch * 100;
    moSbdMessage.roll = roll * 100;

    // Add to statistics object
    //pitchStats.add();
    //rollStats.add();

    DEBUG_PRINTLN("done.");

    // Print debug info
    //DEBUG_PRINT(F("pitch: ")); DEBUG_PRINT_DEC(pitch, 2);
    //DEBUG_PRINT(F(" roll: ")); DEBUG_PRINTLN_DEC(roll, 2);

  }
  else
  {
    DEBUG_PRINTLN("Warning - LSM303 offline!");
  }

  // Stop loop timer
  timer.readLsm303 = millis() - loopStartTime;
}

// ----------------------------------------------------------------------------
//  Vaisala HMP60 Humidity and Temperature Probe
// -----------------------------------------------------
// Colour     Pin     Description
// -----------------------------------------------------
// Brown      12V     Power (5 - 28V)
// White      A3      CH1: Relative humidity (0 - 2.5V)
// Blue       GND     Ground
// Black      A4      CH2: Temperature (0 - 2.5V)
// Shield     GND     Earth ground
// ----------------------------------------------------------------------------
 /*
 void readHmp60()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINT("Info - Reading HMP60...");

  // Note: A startup delay of 4 s is recommended at 12 V and 2 s at 5 V
  myDelay(4000);

  // Perform analog readings
  (void)analogRead(PIN_TEMP);
  float sensorValue1 = analogRead(PIN_TEMP); // External temperature
  (void)analogRead(PIN_HUMID);
  float sensorValue2 = analogRead(PIN_HUMID); // External humidity

  // Map voltages to sensor ranges
  temperatureExt = mapFloat(sensorValue1, 0, 3103, -60, 40);  // Map temperature from 0-2.5 V to -60 to 40°C
  humidityExt = mapFloat(sensorValue2, 0, 3103, 0, 100);      // Map humidity 0-2.5 V to 0 to 100%

  // Calculate measured voltages
  float voltage1 = sensorValue1 * (3.3 / 4095.0);
  float voltage2 = sensorValue2 * (3.3 / 4095.0);

  DEBUG_PRINTLN("done.");

#if CALIBRATE
  // Print calibration data
  DEBUG_PRINT(F("temperatureExt: ")); DEBUG_PRINT(sensorValue1); DEBUG_PRINT(F(",")); DEBUG_PRINT_DEC(voltage1, 4); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(temperatureExt, 2);
  DEBUG_PRINT(F("humidityExt: ")); DEBUG_PRINT(sensorValue2); DEBUG_PRINT(F(",")); DEBUG_PRINT_DEC(voltage2, 4); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(humidityExt, 2);
#endif

  // Add to statistics object
  temperatureExtStats.add(temperatureExt);
  humidityExtStats.add(humidityExt);

  // Stop loop timer
  timer.readHmp60 = millis() - loopStartTime;
}
*/

// ----------------------------------------------------------------------------
// Apogee SP-212 Pyranometer 
// -----------------------------------------------------
// Colour    Pin        Description
// -----------------------------------------------------
// White     A1,A2      Positive (signal from sensor)
// Red       5V         Input Power 5-24 V DC
// Black     GND        Ground (from sensor signal and output power)
// Clear     GND        Shield/Ground
// ----------------------------------------------------------------------------
//Updward Facing 
void readSp212_1()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINT("Info - Reading SP212_1...");

  // Perform analog readings
  // (void)analogRead(PIN_SP212_1);
  float sensorValue = analogRead(PIN_SP212_1); // Incoming Soar Radiation 

  // Map voltages to sensor ranges

  float shortwave1 = mapFloat(sensorValue, 0, 1250, 0, 1000); // Map solar irradiance from 0-1250 mV to 0 to 1000 W m^2 //this is not working in testing need to troubleshoot 

  // Calculate measured voltages
  //float shortwave1 = sensorValue * (0.8); // multiply by 0.8 to get W/m^2 per documentation for this sensor 
    float voltage = sensorValue * (3.3 / 4095.0);

  DEBUG_PRINTLN("done.");

  // Print debug info
  DEBUG_PRINT(F("solar: ")); DEBUG_PRINT(shortwave1); DEBUG_PRINT(F("W/m^2, ")); DEBUG_PRINT(sensorValue); DEBUG_PRINTLN(F("mV,")); 

  // Add to statistics object
  shortwave1Stats.add(shortwave1);

  // Stop loop timer
  timer.readSp212_1 = millis() - loopStartTime;
}
//Downward Facing 
void readSp212_2()
{
  // Start loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINT("Info - Reading SP212_2...");

  // Perform analog readings
  //(void)analogRead(PIN_SP212_2);

  float sensorValue2 = analogRead(PIN_SP212_2); // Incoming Soar Radiation 

  // Map voltages to sensor ranges
  float shortwave2 = mapFloat(sensorValue2, 0, 1250, 0, 1000); // Map solar irradiance from 0-1250 mV to 0 to 1000 W m^2 //this inst working need to troubleshoot 

  // Calculate measured voltages
 // float shortwave2 = sensorValue2 * (0.8);
   float voltage = sensorValue * (3.3 / 4095.0);

  DEBUG_PRINTLN("done.");

  // Print debug info
  DEBUG_PRINT(F("solar: ")); DEBUG_PRINT(shortwave2); DEBUG_PRINT(F("W/m^2, ")); DEBUG_PRINT(sensorValue2); DEBUG_PRINTLN(F("mV,")); 

  // Add to statistics object
  shortwave2Stats.add(shortwave2);

  // Stop loop timer
  timer.readSp212_2 = millis() - loopStartTime;
}


// ----------------------------------------------------------------------------
// R.M. Young Wind Monitor 5103L (4-20 mA)
// 150 Ohm 0.1% resistor
// Voltage range: 0.5995 - 2.9675 V
//
// --------------------------------------------------
// Colour     Pin       Description
// --------------------------------------------------
// Black      12V       Wind speed + (WS+)
// Red        A1        Wind speed - (WS-)
// White      12V       Wind direction + (WD+
// Green      A2        Wind direction - (WD-)
// Shield     GND       Earth ground
//
// ----------------------------------------------------------------------------
 /*
 void read5103L()
{
  unsigned int loopStartTime = millis();

  DEBUG_PRINT("Info - Reading 5103L...");

  // Measure wind speed and direction
  (void)analogRead(PIN_WIND_SPEED);
  float sensorValue1 = analogRead(PIN_WIND_SPEED); // Read analog wind speed value
  (void)analogRead(PIN_WIND_DIR);
  float sensorValue2 = analogRead(PIN_WIND_DIR); // Read analog wind direction value

  // Map wind speed and direction analogue values to
  windSpeed = mapFloat(sensorValue1, 745, 3724, 0, 100); // 0-100 m/s range
  windDirection = mapFloat(sensorValue2, 745, 3724, 0, 360); // 0-360 range

  DEBUG_PRINTLN("done.");

#if CALIBRATE
  // Calculate measured voltages
  float voltage1 = sensorValue1 * (3.3 / 4095.0);
  float voltage2 = sensorValue2 * (3.3 / 4095.0);

  // Print calibration data
  DEBUG_PRINT(F("windSpeed: ")); DEBUG_PRINT_DEC(voltage1, 4); DEBUG_PRINT(F(",")); DEBUG_PRINT(sensorValue1); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(windSpeed, 2);
  DEBUG_PRINT(F("windDirection: ")); DEBUG_PRINT_DEC(voltage2, 4); DEBUG_PRINT(F(",")); DEBUG_PRINT(sensorValue2); DEBUG_PRINT(F(",")); DEBUG_PRINTLN_DEC(windDirection, 2);
#endif

  // Check and update wind gust and direction
  if ((windSpeed > 0) && (windSpeed > windGustSpeed))
  {
    windGustSpeed = windSpeed;
    windGustDirection = windDirection;
  }

  // Calculate wind speed and direction vectors
  // For more information see:
  // http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
  float windDirectionRadians = windDirection * DEG_TO_RAD;  // Convert wind direction from degrees to radians
  float u = -1.0 * windSpeed * sin(windDirectionRadians);   // Magnitude of east-west component (u) of vector winds
  float v = -1.0 * windSpeed * cos(windDirectionRadians);   // Magnitude of north-south component (v) of vector winds

  // Write data to union
  moSbdMessage.windGustSpeed = windGustSpeed * 100;
  moSbdMessage.windGustDirection = windGustDirection * 10;

  // Add to wind statistics
  windSpeedStats.add(windSpeed);
  uStats.add(u);
  vStats.add(v);

  // Stop loop timer
  timer.read5103L = millis() - loopStartTime;
}
*/
// ----------------------------------------------------------------------------
// Davis Instruments 7911 Anemometer
// ------------------------------
// Colour   Pin     Description
// ------------------------------
// Black    A1      Wind speed
// Green    A2      Wind direction
// Yellow   5V      Power
// Red      GND     Ground
// ----------------------------------------------------------------------------
 /*
 void read7911()
{
  uint32_t loopStartTime = millis();

  DEBUG_PRINT("Info - Reading 7911...");

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

  // Measure wind direction
  (void)analogRead(PIN_WIND_DIR);
  windDirection = analogRead(PIN_WIND_DIR); // Raw analog wind direction value
  windDirection = map(windDirection, 0, 4095, 0, 359); // Map wind direction to degrees (0-360°)

  // Correct for negative wind direction values
  if (windDirection > 360)
    windDirection -= 360;
  if (windDirection < 0)
    windDirection += 360;

  if (windSpeed == 0)
  {
    // windDirection = 0.0; // Comment 2023-06-30: Perhaps not best practice?
  }

  // Check and update wind gust speed and direction
  if ((windSpeed > 0) && (windSpeed > windGustSpeed))
  {
    windGustSpeed = windSpeed;
    windGustDirection = windDirection;
  }

  // Calculate wind speed and direction vectors
  // http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
  float windDirectionRadians = windDirection * DEG_TO_RAD;  // Convert wind direction from degrees to radians
  float u = -1.0 * windSpeed * sin(windDirectionRadians);   // Magnitude of east-west component (u) of vector winds
  float v = -1.0 * windSpeed * cos(windDirectionRadians);   // Magnitude of north-south component (v) of vector winds

  // Write data to union
  moSbdMessage.windGustSpeed = windGustSpeed * 100;
  moSbdMessage.windGustDirection = windGustDirection * 10;

  // Add to wind statistics
  windSpeedStats.add(windSpeed);
  uStats.add(u);
  vStats.add(v);

  DEBUG_PRINTLN("done.");

  // Print debug info
  //DEBUG_PRINT(F("Wind Speed: ")); DEBUG_PRINTLN(windSpeed);
  //DEBUG_PRINT(F("Wind Direction: ")); DEBUG_PRINTLN(windDirection);

  // Stop loop timer
  timer.read7911 = millis() - loopStartTime;
}

// Interrupt service routine (ISR) for Davis Instruments 7911 anemometer
// wind speed measurement
void windSpeedIsr()
{
  revolutions++;
}

// Calculate mean wind speed and direction from vector components
// For more information see:
// http://tornado.sfsu.edu/geosciences/classes/m430/Wind/WindDirection.html
void windVectors()
{
  // Calculate resultant mean wind speed
  float rvWindSpeed = sqrt(sq(uStats.average()) + sq(vStats.average()));

  DEBUG_PRINT("uStats.average(): "); printTab(1); DEBUG_PRINTLN(uStats.average());
  DEBUG_PRINT("vStats.average(): "); printTab(1); DEBUG_PRINTLN(vStats.average());

  // Calculate resultant mean wind direction
  float rvWindDirection = atan2(-1.0 * uStats.average(), -1.0 * vStats.average());
  rvWindDirection *= RAD_TO_DEG;  // Convert from radians to degrees

  DEBUG_PRINT("rvWindSpeed: "); printTab(2); DEBUG_PRINTLN(rvWindSpeed);
  DEBUG_PRINT("rvWindDirection: "); printTab(1); DEBUG_PRINTLN(rvWindDirection);

  // To do: Check if necessary
  if (rvWindDirection < 0)
    rvWindDirection += 360;

  // Zero wind direction if wind speed is zero
  // Note: atan2 can be undefined if u and v vectors are zero
  if (rvWindSpeed == 0)
    rvWindDirection = 0;

  // Write data to union
  moSbdMessage.windSpeed = rvWindSpeed * 100;         // Resultant mean wind speed (m/s)
  moSbdMessage.windDirection = rvWindDirection * 10;  // Resultant mean wind direction (°)
}
*/
// ----------------------------------------------------------------------------
// Adafruit Temperature Humidity Sensor (SHT-30) 
// ------------------------------
// Colour     Pin     Description
// ------------------------------
// Brown/red       3.3V     Power
// Black           GND      Ground
// Yellow          SCL      Clock
// Blue/Green      SDA      Data
// ----------------------------------------------------------------------------
// default I2C - 0x44
//----------------------------------------------------------------------------
//adapted from previous SHT-31 program 

void readsht30(){
  // Start the loop timer
  unsigned long loopStartTime = millis();

  DEBUG_PRINT("Info - Reading SHT30...");
  sht31.begin(0x44);
  // Add delay
  myDelay(100);

  // Read sensor
  temperatureExt = sht31.readTemperature();
  humidityExt = sht31.readHumidity();

  // Add to statistics object
  temperatureExtStats.add(temperatureExt);
  humidityExtStats.add(humidityExt);

  // Print debug info
 // DEBUG_PRINT(F("Temperature: ")); DEBUG_PRINT(temperatureExt); DEBUG_PRINTLN(" C");
  //DEBUG_PRINT(F("Humidity: ")); DEBUG_PRINT(humidityExt); DEBUG_PRINTLN("%");

DEBUG_PRINTLN("Done.");

  // Stop the loop timer
  timer.readsht30 = millis() - loopStartTime;
}


// ----------------------------------------------------------------------------
// MaxBotix MB7354 HRXL-MaxSonar-WRS5
// https://www.maxbotix.com/ultrasonic_sensors/mb7354.htm
// --------------------------------------------------
// Colour    Pin    Description             Notes
// --------------------------------------------------
// White          Temperature Sensor      Not connected
// Orange   5    Pulse Width Output      Pulse width in 
// Brown          Analog Voltage Output   Analog In (not connected)
// Green    9    Ranging Start/Stop      Not connected
// Blue           Serial Output           Not connected
// Red      5V    Vcc                     5V
// Black    GND   GND                     GND
//
// ----------------------------------------------------------------------------
// Read Maxbotix distance to surface
void readMxBtx() {
  // Wake sensor
  digitalWrite(PIN_MB_sleep, HIGH);
  delay(100);

  // Start loop timer
  unsigned int loopStartTime = millis();

   DEBUG_PRINT("Info - Reading MaxBotix...");
  
  // Create a temporary Statistic array to hold the maxbotix measurements
  Statistic Maxbotix;

  // Create temporary variables
  unsigned int z, z_av, z_std, z_max, z_min, z_nan;
  z = 0;
  z_av = 0;
  z_std = 0;
  z_max = 0;
  z_min = 0;
  z_nan = 0;
  
  // Get 30 z readings in mm, filtering out reading 50 mm
  // above/below sensor minumum/maximum readings
  for(byte i = 0; i < 30; i++) {
    z = pulseIn(PIN_MB_pw, HIGH); // Read distance to snow surface
    
    if (z > 550 && z < 4950) { // Filter readings
      Maxbotix.add(z); // Add good readings to stats array
    }
    else {
      z_nan += 1; // Count bad readings
    }
    delay(100); // Delay 0.1 secs between readings
  }

  // Get stats from the Maxbotix array in mm
  z_av = Maxbotix.average(), 0;
  z_std = Maxbotix.pop_stdev(), 0;
  z_max = Maxbotix.maximum(), 0;
  z_min = Maxbotix.minimum(), 0;
    
  // Deal with issue of a maximum long number in the instance of no
  // readings within filtered range
  if (z_av > 5000) {
    z_av = 0;
  }
  if (z_std > 5000) {
    z_std = 0;
  }
  if (z_max > 5000) {
  z_max = 0;
  }
  if (z_min > 5000) {
  z_min = 0;
  }
  
  // Add sample stats to global arrays
  MaxbotixStats_av.add(z_av);
  MaxbotixStats_std.add(z_std);
  MaxbotixStats_max.add(z_max);
  MaxbotixStats_min.add(z_min);
  MaxbotixStats_nan.add(z_nan);

  // Add to sample variables
  distMaxbotix_av  = z_av;
  distMaxbotix_std = z_std;
  distMaxbotix_max = z_max;
  distMaxbotix_min = z_min;
  distMaxbotix_nan = z_nan;

  // Clear local array
  Maxbotix.clear();

   // Stop loop timer
  timer.readMxBtx = millis() - loopStartTime;

  DEBUG_PRINTLN("done.");

  // Print debug info
  DEBUG_PRINT(F("Distance: ")); DEBUG_PRINT(z_av); DEBUG_PRINT(F(" mm, Nan Count:")); DEBUG_PRINT(z_nan); DEBUG_PRINTLN(F(","));


  
  // Sleep sensor
  digitalWrite(PIN_MB_sleep, LOW);
  delay(100);
  
}
