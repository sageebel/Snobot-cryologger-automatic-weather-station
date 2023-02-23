// Configure microSD
void configureSd()
{
#if LOGGING
  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if microSD has been initialized
  if (!online.microSd)
  {
    // Initialize microSD
    if (!sd.begin(PIN_MICROSD_CS, SD_SCK_MHZ(4)))
    {
      DEBUG_PRINTLN("Warning - microSD failed to initialize. Reattempting...");

      // Delay between initialization attempts
      myDelay(2000);

      if (!sd.begin(PIN_MICROSD_CS, SD_SCK_MHZ(4)))
      {
        online.microSd = false;
        DEBUG_PRINTLN("Warning - microSD failed to initialize.");

        /*
          while (1)
          {
          // Force WDT to reset system
          blinkLed(PIN_LED_RED, 2, 250);
          delay(2000);
          }
        */
      }
      else
      {
        online.microSd = true; // Set flag
        DEBUG_PRINTLN("Info - microSD initialized.");
      }
    }
    else
    {
      online.microSd = true; // Set flag
      DEBUG_PRINTLN("Info - microSD initialized.");
    }
  }
  else
  {
    DEBUG_PRINTLN("Info - microSD already initialized.");
    return;
  }

  // Stop the loop timer
  timer.configMicroSd = millis() - loopStartTime;
#endif
}

// Create timestamped log file
void createLogFile()
{
#if LOGGING

  // Get timestamp log file name
  sprintf(logFileName, "AWS_%d_20%02d%02d%02d_%02d%02d%02d.csv",
          SNOBOT_ID, rtc.getYear(), rtc.getMonth(), rtc.getDay(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());

  DEBUG_PRINT("Info - Log file name: "); DEBUG_PRINTLN(logFileName);

  // Check if log file is open
  if (logFile.isOpen())
    logFile.close();

  // Create a new log file and open for writing
  // O_CREAT  - Create the file if it does not exist
  // O_APPEND - Seek to the end of the file prior to each write
  // O_WRITE  - Open the file for writing
  if (!logFile.open(logFileName, O_CREAT | O_APPEND | O_WRITE))
  {
    DEBUG_PRINT("Warning - Failed to create log file"); DEBUG_PRINTLN(logFileName);
    return;
  }
  else
  {
    DEBUG_PRINT("Info - Created log file: "); DEBUG_PRINTLN(logFileName);
  }

  if (!logFile.isOpen())
  {
    DEBUG_PRINTLN(F("Unable to open file"));
  }

  // Read RTC date and time

  // Update file create timestamp
  updateFileCreate(&logFile);

  // Write header to file //need to update this 
  DEBUG_PRINTln("sample,datetime,voltage,temperature_int,humidity_int,pressure_int,temperature_ext,"
                  "humidity_ext,distMaxbotix_av,distMaxbotix_std,distMaxbotix_max,distMaxbotix_min,distMaxbotix_nan,pitch,roll,sw_1,sw_2,soil_1,soiil_2,latitude,longitude,satellites,hdop,"
                  "online_microSd,online_readBme280,online_readLsm303,timer_readRtc,timer_readBattery,timer_configMicroSd,"
                  "timer_readGnss,timer_bme280,timer_lsm303"
                  "sampleInterval,averageInterval,transmitInterval,retransmitLimit,gnssTimeout");

  // Close log file
  logFile.close();
#endif
}

void checkLogFile()
{
  // Record log file tracker the first time program runs
  if (firstTimeFlag)
  {
    if (loggingMode == 1) // Daily
      currentLogFile = rtc.getDay();
    else if (loggingMode == 2) // Monthly
      currentLogFile = rtc.getMonth();
    else if (loggingMode == 3) // Yearly
      currentLogFile = rtc.getYear();
    else // Default to monthly
      currentLogFile = rtc.getMonth();
  }

  // Update log file tracker
  if (loggingMode == 1) // Daily
    newLogFile = rtc.getDay();
  else if (loggingMode == 2) // Monthly
    newLogFile = rtc.getMonth();
  else if (loggingMode == 3) // Yearly
    newLogFile = rtc.getYear();
  else // Default to monthly
    newLogFile = rtc.getMonth();
}

// Write data to log file
void logData()
{
#if LOGGING
  // Configure microSD
  configureSd();

  // Start loop timer
  unsigned long loopStartTime = millis();

  // Check if microSD is online
  if (online.microSd)
  {
    // Check if new log file should be created
    checkLogFile();
    if (currentLogFile != newLogFile)
    {
      createLogFile();
      currentLogFile = newLogFile;
      samplesSaved = 0;
    }

    // Write to microSD card
    if (logFile.open(logFileName, O_APPEND | O_WRITE))
    {
      // Sensor information
      samplesSaved++; //  Increment sample count
      DEBUG_PRINT(samplesSaved);        DEBUG_PRINT(",");
      DEBUG_PRINT(dateTime);            DEBUG_PRINT(",");
      DEBUG_PRINT(voltage);             DEBUG_PRINT(",");
      DEBUG_PRINT(temperatureInt);      DEBUG_PRINT(",");
      DEBUG_PRINT(humidityInt);         DEBUG_PRINT(",");
      DEBUG_PRINT(pressureInt);         DEBUG_PRINT(",");
      DEBUG_PRINT(temperatureExt);      DEBUG_PRINT(",");
      DEBUG_PRINT(humidityExt);         DEBUG_PRINT(",");
      DEBUG_PRINT(distMaxbotix_av);     DEBUG_PRINT(",");   
      DEBUG_PRINT(distMaxbotix_std);    DEBUG_PRINT(","); 
      DEBUG_PRINT(distMaxbotix_max);    DEBUG_PRINT(","); 
      DEBUG_PRINT(distMaxbotix_min);    DEBUG_PRINT(","); 
      DEBUG_PRINT(distMaxbotix_nan);    DEBUG_PRINT(",");    
      DEBUG_PRINT(pitch);               DEBUG_PRINT(",");
      DEBUG_PRINT(roll);                DEBUG_PRINT(",");
      DEBUG_PRINT(shortwave1);          DEBUG_PRINT(",");
      DEBUG_PRINT(shortwave2);          DEBUG_PRINT(",");
      DEBUG_PRINT(soilmoist1);          DEBUG_PRINT(",");
      DEBUG_PRINT(soilmoist2);          DEBUG_PRINT(",");
      DEBUG_PRINT(latitude, 6);         DEBUG_PRINT(",");
      DEBUG_PRINT(longitude, 6);        DEBUG_PRINT(",");
      DEBUG_PRINT(satellites);          DEBUG_PRINT(",");
      DEBUG_PRINT(hdop);                DEBUG_PRINT(",");

      // Online information
      DEBUG_PRINT(online.microSd);      DEBUG_PRINT(",");
      DEBUG_PRINT(online.bme280);       DEBUG_PRINT(",");
      DEBUG_PRINT(online.lsm303);       DEBUG_PRINT(",");

      // Timer information
      DEBUG_PRINT(timer.readRtc);       DEBUG_PRINT(",");
      DEBUG_PRINT(timer.readBattery);   DEBUG_PRINT(",");
      DEBUG_PRINT(timer.configMicroSd); DEBUG_PRINT(",");
      DEBUG_PRINT(timer.readGnss);      DEBUG_PRINT(",");
      DEBUG_PRINT(timer.readBme280);    DEBUG_PRINT(",");
      DEBUG_PRINT(timer.readLsm303);    DEBUG_PRINT(",");
      // DEBUG_PRINT(timer.readHmp60);     DEBUG_PRINT(",");
      // DEBUG_PRINT(timer.read5103L);     DEBUG_PRINT(",");
      //DEBUG_PRINT(timer.readSp212_1);     DEBUG_PRINT(",");
      // DEBUG_PRINT(timer.iridium);       DEBUG_PRINT(",");

      // Debugging information
      DEBUG_PRINT(transmitStatus);      DEBUG_PRINT(",");
      DEBUG_PRINT(rtcDrift);            DEBUG_PRINT(",");
      DEBUG_PRINT(freeRam());           DEBUG_PRINT(",");

      // Sampling information
      DEBUG_PRINT(sampleInterval);      DEBUG_PRINT(",");
      DEBUG_PRINT(averageInterval);     DEBUG_PRINT(",");
      DEBUG_PRINT(transmitInterval);    DEBUG_PRINT(",");
      DEBUG_PRINT(retransmitLimit);     DEBUG_PRINT(",");
      DEBUG_PRINT(gnssTimeout);         DEBUG_PRINT(",");
      // DEBUG_PRINTln(iridiumTimeout);      

      // Update file access timestamps
      updateFileAccess(&logFile);

      // Force data to SD and update the directory entry to avoid data loss
      if (!logFile.sync())
      {
        DEBUG_PRINTLN(F("Warning - microSD sync error!"));
      }

      // Close the log file
      if (!logFile.close())
      {
        DEBUG_PRINTLN("Warning - Failed to close log file!");
        //closeFailCounter++; // Count number of failed file closes
      }

#if DEBUG
      // Print logged data to Serial Monitor

      DEBUG_PRINT("Info - Logging data to: "); DEBUG_PRINTLN(logFileName);
      DEBUG_PRINT(samplesSaved);        DEBUG_PRINT(",");
      DEBUG_PRINT(dateTime);            DEBUG_PRINT(",");
      DEBUG_PRINT(voltage);             DEBUG_PRINT(",");
      DEBUG_PRINT(temperatureInt);      DEBUG_PRINT(",");
      DEBUG_PRINT(humidityInt);         DEBUG_PRINT(",");
      DEBUG_PRINT(pressureInt);         DEBUG_PRINT(",");
      DEBUG_PRINT(temperatureExt);      DEBUG_PRINT(",");
      DEBUG_PRINT(humidityExt);         DEBUG_PRINT(",");
      DEBUG_PRINT(pitch);               DEBUG_PRINT(",");
      DEBUG_PRINT(roll);                DEBUG_PRINT(",");      
      // DEBUG_PRINT(windSpeed);           DEBUG_PRINT(",");
      // DEBUG_PRINT(windDirection);       DEBUG_PRINT(",");
      //DEBUG_PRINT(solar);               DEBUG_PRINT(",");
      DEBUG_PRINT(shortwave1);          DEBUG_PRINT(",");
      DEBUG_PRINT(shortwave2);          DEBUG_PRINT(",");
      DEBUG_PRINT(soilmoist1);          DEBUG_PRINT(",");
      DEBUG_PRINT(soilmoist2);          DEBUG_PRINT(",");
      DEBUG_PRINT(latitude, 6);         DEBUG_PRINT(",");
      DEBUG_PRINT(longitude, 6);        DEBUG_PRINT(",");
      DEBUG_PRINT(satellites);          DEBUG_PRINT(",");
      DEBUG_PRINT(hdop);                DEBUG_PRINT(",");
      DEBUG_PRINT_DEC(latitude, 6);     DEBUG_PRINT(",");
      DEBUG_PRINT_DEC(longitude, 6);    DEBUG_PRINT(",");
      DEBUG_PRINT(satellites);          DEBUG_PRINT(",");
      DEBUG_PRINT(hdop);                DEBUG_PRINT(",");

      // Online information
      DEBUG_PRINT(online.microSd);      DEBUG_PRINT(",");
      DEBUG_PRINT(online.bme280);       DEBUG_PRINT(",");
      DEBUG_PRINT(online.lsm303);       DEBUG_PRINT(",");

      // Timer information
      DEBUG_PRINT(timer.readRtc);       DEBUG_PRINT(",");
      DEBUG_PRINT(timer.readBattery);   DEBUG_PRINT(",");
      DEBUG_PRINT(timer.configMicroSd); DEBUG_PRINT(",");
      DEBUG_PRINT(timer.readGnss);      DEBUG_PRINT(",");
      DEBUG_PRINT(timer.readBme280);    DEBUG_PRINT(",");
      DEBUG_PRINT(timer.readLsm303);    DEBUG_PRINT(",");
      // DEBUG_PRINT(timer.readHmp60);     DEBUG_PRINT(",");
      // DEBUG_PRINT(timer.read5103L);     DEBUG_PRINT(",");
      // DEBUG_PRINT(timer.readSp212);     DEBUG_PRINT(",");
      // DEBUG_PRINT(timer.iridium);       DEBUG_PRINT(",");

      // Debugging information
      // DEBUG_PRINT(transmitStatus);      DEBUG_PRINT(",");
      // DEBUG_PRINT(rtcDrift);            DEBUG_PRINT(",");
      // DEBUG_PRINT(freeRam());

      // Sampling information
      DEBUG_PRINT(sampleInterval);      DEBUG_PRINT(",");
      DEBUG_PRINT(averageInterval);     DEBUG_PRINT(",");
      // DEBUG_PRINT(transmitInterval);    DEBUG_PRINT(",");
      // DEBUG_PRINT(retransmitLimit);     DEBUG_PRINT(",");
      // DEBUG_PRINT(gnssTimeout);         DEBUG_PRINT(",");
      // DEBUG_PRINTLN(iridiumTimeout);
#endif
      blinkLed(PIN_LED_GREEN, 2, 100);
    }
    else
    {
      DEBUG_PRINTLN(F("Warning - Unable to open file!"));
    }
  }
  else
  {
    DEBUG_PRINTLN(F("Warning - microSD is offline!"));
  }

  // Stop the loop timer
  timer.writeMicroSd = millis() - loopStartTime;
#endif
}


// Update the file create timestamp
void updateFileCreate(FsFile *dataFile)
{
  if (!dataFile->timestamp(T_CREATE, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()))
  {
    DEBUG_PRINT("Warning - Could not update file create timestamp.");
  }
}

// Update the file access and write timestamps
void updateFileAccess(FsFile *dataFile)
{
  if (!dataFile->timestamp(T_ACCESS, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()))
  {
    DEBUG_PRINTLN("Warning - Could not update file access timestamp.");
  }
  if (!dataFile->timestamp(T_WRITE, (rtc.getYear() + 2000), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds()))
  {
    DEBUG_PRINTLN("Warning - Could not update file write timestamp.");
  }
}
