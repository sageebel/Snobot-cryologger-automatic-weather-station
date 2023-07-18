/*
  Title:    METER TEROS 10 Test Code
  Date:     July 18,2023
  Author:   Sage Ebel 

  Sensor:
  - METER Group TEROS 10 Soil Moisture Sensor 
  https://www.apogeeinstruments.com/sp-212-ss-amplified-0-2-5-volt-pyranometer/

  Notes: 
  - 1,000 to 2500 mV analog output 
  -Volumetric Water Content given by: VWC = 4.824*10^-10 * (mV^3) - 2.278*10^-6 * (mV^2) + 3.898*10^-3 * (mV) - 2.154
  
  Wiring Diagram:
  -----------------------------------------------------
  Colour    Pin        Description
  -----------------------------------------------------
  Orange    AX         Positive (signal from sensor)
  Brown     5V         Input Power 3-15 VDC
  Clear     GND        Shield/Ground

*/

#define PIN_SOIL_1   A3
#define PIN_SOIL_2   A4

void setup()
{
  pinMode(PIN_5V_EN, OUTPUT);
  digitalWrite(PIN_5V_EN, HIGH);

  Serial.begin(115200);
  //while (!Serial);

}

void loop()
{
  (void) analogRead(PIN_SOIL_1);
  (void)analogRead(PIN_SOIL_2);
  float sensorValue_1 = analogRead(PIN_SOIL_1); // voltage across forks of sensor in mV 
  float sensorValue_2 = analogRead(PIN_SOIL_2); 
  float soilmoist1 = 4.824*10^-10 * (sensorValue_1) - 2.278*10^-6 * (sensorValue_1) + 3.898*10^-3 * (sensorValue_1) - 2.154
  float soilmoist2 = 4.824*10^-10 * (sensorValue_2) - 2.278*10^-6 * (sensorValue_2) + 3.898*10^-3 * (sensorValue_2) - 2.154

  Serial.print(soilmoist1); Serial.print(F(",")); Serial.print(sensorValue_1, 4); Serial.println(F(" mV,"));
  Serial.print(soilmoist2); Serial.print(F(",")); Serial.print(sensorValue_2, 4); Serial.println(F(" mV,"));
  delay(1000);
}
