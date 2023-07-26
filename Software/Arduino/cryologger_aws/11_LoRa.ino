//LoRa Radio Communication 



// Set-up to talk to LoRA
void talkToRadio(){
  digitalWrite(PIN_MICROSD_CS, HIGH); //turn on the SD card pin
  digitalWrite(PIN_RFM95_CS, LOW);        //turn off the chip select pin on the lora 
  delay(1);
}

// Set-up to talk to SD
void talkToSD(){
  digitalWrite(PIN_RFM95_CS, HIGH);     //turn on the chip select pin on the lora 
  digitalWrite(PIN_MICROSD_CS, LOW);        //turn off the sd card pin 
  delay(1);
}

// Send LoRa line
//#if NODE_STATION
  void LoRa_send(){
    unsigned int loopStartTime = millis();
    uint8_t from;

    talkToRadio();
    packetnum++;
    
    #if DEBUG
    DEBUG_PRINTLN("Attempting to Transmit Message to Base via LoRa..."); // Send a message to rf95_server
    #endif

    // Send a message to manager_server
    if (manager.sendtoWait((uint8_t *)&tx_message, sizeof(tx_message), BASE_ADDRESS))
    {
      // Now wait for a reply from the server
      uint8_t len = sizeof(buf);
      
      if (manager.recvfromAckTimeout(buf, &len, 4000, &from))
      {
        DEBUG_PRINT("got reply from base station #:");
        DEBUG_PRINT_HEX(from, HEX);
        DEBUG_PRINT(": ");
        DEBUG_PRINTLN((char*)buf);
        loraRxFlag = true;
      }
      else
      {
        DEBUG_PRINTLN("No reply, is the base station running?");
      }
    }
    else
      DEBUG_PRINTLN("sendtoWait failed");

    DEBUG_PRINT("RSSI = "); DEBUG_PRINTLN(rf95.lastRssi());
    
    unsigned int loraLoopTime = millis() - loopStartTime;
    Serial.print("LoRa_send() function execution: "); Serial.print(loraLoopTime); Serial.println(F(" ms"));
  }
//#endif

//#if BASE_STATION //listen for LoRa messages and write them to the SD card 
  // Write received data from LoRa to RockBlock buffer

  void LoRa_receive()
  {
    // Volatile boolean for if a message is received
    volatile bool rxFlag = false;
    uint8_t from;

    talkToRadio();

    if (manager.available())
    {
      // Wait for a message addressed to us from the client
      uint8_t len = sizeof(buf);
      
      if (manager.recvfromAck(buf, &len, &from))
      {
        DEBUG_PRINT("Got data from SnowBot #");
        DEBUG_PRINT_HEX(from, HEX);
        DEBUG_PRINT(": ");
        DEBUG_PRINTLN((char*)buf);
        rxFlag = true; //Set rxFlag to true
        
        // Send a reply back to the originator client
        if (!manager.sendtoWait(rx_reply, sizeof(rx_reply), from))
        {
            DEBUG_PRINTLN("sendtoWait failed");
            rxFlag = false; //Set rxFlag to false if sendtoWait fails
        }
      }
      else 
      {
        (DEBUG_PRINTLN("Can't find a message"));
      }
    }

    if (rxFlag) 
    {
      // Add received buf into rx_message
      memcpy(&rx_message, buf, sizeof(rx_message)); // Copy received into rx_message
      
      // Write received rx message to the RockBlock buffer
      writeBuffer_rx();
    
      // Write received message to SD file
      talkToSD(); // Talk to SD card
    
      // write incoming data to SD card 

      logData(); 

      DEBUG_PRINT("Logged Data From Snobot #");
      DEBUG_PRINT_HEX(from, HEX);
      DEBUG_PRINT(": ");
      DEBUG_PRINTLN((char*)buf);

    
    }
    // Clear data stored in rx_message
    memset(rx_message.bytes, 0x00, sizeof(rx_message));

    // Clear rxFlag
    rxFlag = false;
  }
//#endif