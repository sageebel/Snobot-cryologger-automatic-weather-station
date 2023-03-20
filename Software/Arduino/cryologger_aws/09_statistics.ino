// Calculate statistics
void calculateStats()
{
  // Write data to union
  moSbdMessage.temperatureInt = temperatureIntStats.average()   * 100;          // Mean internal temperature (°C)
  moSbdMessage.humidityInt    = humidityIntStats.average()      * 100;          // Mean internal humidity (%)
  moSbdMessage.pressureInt    = (pressureIntStats.average()     - 850) * 100;   // Mean internal pressure (hPa)
  moSbdMessage.temperatureExt = temperatureExtStats.average()   * 100;          // Mean external temperature (°C)
  moSbdMessage.humidityExt    = humidityExtStats.average()      * 100;          // Mean external humidity (%)
  //moSbdMessage.solar          = solarStats.average()            * 10;           // Mean solar irradiance (W m-2)
  moSbdMessage.voltage        = batteryStats.average()          * 100;          // Mean battery voltage (V)

  // Calculate mean wind speed and direction vectors
  // windVectors();

  // Clear all statistics objects
  clearStats();

  // Clear wind gust speed and direction maximums
  windGustSpeed = 0;
  windGustDirection = 0;
}

// Clear statistics objects
void clearStats()
{
  batteryStats.clear();
  temperatureIntStats.clear();
  humidityIntStats.clear();
  pressureIntStats.clear();
  temperatureExtStats.clear();
  //solarStats.clear();
  humidityExtStats.clear();
  windSpeedStats.clear();
  uStats.clear();
  vStats.clear();
}

// Print statistics
void printStats()
{
  printLine();
  DEBUG_PRINTLN(F("Statistics"));
  printLine();
  DEBUG_PRINT("Datetime: ");      printTab(1);  printDateTime();
  DEBUG_PRINT(F("Voltage"));      printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(batteryStats.count());            printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(batteryStats.minimum());          printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(batteryStats.maximum());          printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(batteryStats.average());
  DEBUG_PRINT(F("Temp Int"));                                                   printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(temperatureIntStats.count());     printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(temperatureIntStats.minimum());   printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(temperatureIntStats.maximum());   printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(temperatureIntStats.average());
  DEBUG_PRINT(F("Humidity Int"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(humidityIntStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(humidityIntStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(humidityIntStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(humidityIntStats.average());
  DEBUG_PRINT(F("Pressure Int"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(pressureIntStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(pressureIntStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(pressureIntStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(pressureIntStats.average());
  DEBUG_PRINT(F("Temp Ext"));                                                   printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(temperatureExtStats.count());     printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(temperatureExtStats.minimum());   printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(temperatureExtStats.maximum());   printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(temperatureExtStats.average());
  DEBUG_PRINT(F("Humidity Ext"));                                               printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(humidityExtStats.count());        printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(humidityExtStats.minimum());      printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(humidityExtStats.maximum());      printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(humidityExtStats.average());
  DEBUG_PRINT(F("Short Wave 1"));                                               printTab(2);   // why 2 here
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(shortwave1Stats.count());         printTab(1); 
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(shortwave1Stats.minimum());       printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(shortwave1Stats.maximum());       printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(shortwave1Stats.average());
  DEBUG_PRINT(F("Short Wave 2"));                                               printTab(2);   // why 2 here
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(shortwave2Stats.count());         printTab(1); 
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(shortwave2Stats.minimum());       printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(shortwave2Stats.maximum());       printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(shortwave2Stats.average());
  DEBUG_PRINT(F("Soil Moist 1"));                                               printTab(2);   // why 2 here
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(soilmoist1Stats.count());         printTab(1); 
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(soilmoist1Stats.minimum());       printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(soilmoist1Stats.maximum());       printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(soilmoist1Stats.average());
  DEBUG_PRINT(F("Soil Moist 2"));                                               printTab(2);   // why 2 here
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(soilmoist2Stats.count());         printTab(1); 
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(soilmoist2Stats.minimum());       printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(soilmoist2Stats.maximum());       printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(soilmoist2Stats.average());
  DEBUG_PRINT(F("Max Botix"));                                                  printTab(2);   // why 2 here
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(MaxbotixStats_av.count());        printTab(1); 
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(MaxbotixStats_min.minimum());             printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(MaxbotixStats_max.maximum());             printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(MaxbotixStats_av.average());            printTab(1);
  DEBUG_PRINT(F("NAN count: "));  DEBUG_PRINTLN(MaxbotixStats_nan.count());            
  DEBUG_PRINT(F("Wind speed"));   printTab(1);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(windSpeedStats.count());          printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(windSpeedStats.minimum());        printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(windSpeedStats.maximum());        printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(windSpeedStats.average());
  DEBUG_PRINT(F("vn"));                                                         printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(vStats.count());                  printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(vStats.minimum());                printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(vStats.maximum());                printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(vStats.average());
  DEBUG_PRINT(F("ve"));                                                         printTab(2);
  DEBUG_PRINT(F("Samples: "));    DEBUG_PRINT(uStats.count());                  printTab(1);
  DEBUG_PRINT(F("Min: "));        DEBUG_PRINT(uStats.minimum());                printTab(1);
  DEBUG_PRINT(F("Max: "));        DEBUG_PRINT(uStats.maximum());                printTab(1);
  DEBUG_PRINT(F("Mean: "));       DEBUG_PRINTLN(uStats.average());
  DEBUG_PRINT(F("Wind gust speed: "));      printTab(1);  DEBUG_PRINTLN(windGustSpeed);
  DEBUG_PRINT(F("Wind gust direction: "));  printTab(1);  DEBUG_PRINTLN(windGustDirection);
}
