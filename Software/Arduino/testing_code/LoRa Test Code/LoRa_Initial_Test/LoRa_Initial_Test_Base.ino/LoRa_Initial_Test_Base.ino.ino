//LoRa Base Test Code 

// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h> // https://github.com/PaulStoffregen/RadioHead


// Feather M0:
#define RFM95_CS   A3
#define RFM95_RST  A4
#define RFM95_INT  9

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------
#define BASE_STATION   true   // Set if RockBlock is installed and you wish to listen for LoRa messages


// ----------------------------------------------------------------------------
// User defined global variable declarations (Base vs Node) (These Change with each different station. Refer to : https://docs.google.com/spreadsheets/d/1wqMFbQtYPQnuI8aEQJs2P38mY3F423x5D4DhG7ajBnw/edit#gid=548097118)
// ----------------------------------------------------------------------------
char          node_name           = 'h';      // Node group identifier(high = h, mid = m, low = l, north = n)
unsigned int  node_number         = 5;            // Node number
unsigned int  base_station_number = 1;            // Number of snow bot for datagram (100 + node)
unsigned int  total_nodes         = 3;            // Total nodes in the network (excluding base station) 

#define NODE_ADDRESS  node_number         // Node number is local LoRa address
#define BASE_ADDRESS  base_station_number // Number of station that is LoRa base station w/ RockBlock

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT); // rf95(slaveselect, interrupt pin)

// ----------------------------------------------------------------------------
// Datagram Set-up (Manages which Node and Base Station are communicating)
// ----------------------------------------------------------------------------
#if NODE_STATION
RHReliableDatagram manager(rf95, NODE_ADDRESS);  // Class to manage message delivery and receipt, using the driver declared above
#endif

#if BASE_STATION
RHReliableDatagram manager(rf95, BASE_ADDRESS);  // Class to manage message delivery and receipt, using the driver declared above
#endif

// ----------------------------------------------------------------------------
// Unions/structures 
// ----------------------------------------------------------------------------
typedef union {
  char d[20] = "hello base";
} SBD_MO_MESSAGE;

SBD_MO_MESSAGE rx_message;        //Incoming LoRa message 

// ----------------------------------------------------------------------------
// Global Variables 
// ----------------------------------------------------------------------------
int16_t       packetnum;                               // LoRa radio packet number
uint8_t       buf[RH_RF95_MAX_MESSAGE_LEN];            // LoRa radio buffer
uint8_t       len                    = sizeof(buf);    // LoRa radio buffer length
uint8_t       rx_reply[]        = "hello back to you node";   //Send a reply to the node for DEBUGGIN
volatile bool loraRxFlag        = false;               // LoRa message received flag

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa RX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  Serial.println("Feather LoRa Manager Test!");
  if (!manager.init())
    Serial.println("LoRa manager init failed");
  Serial.println("LoRa radio manager init OK!");
  

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop() {

  delay(1000);
    Serial.println("Listening For Messages from Node Station..."); // Waiting to see if there is a message from rf95_server
    uint8_t from;
   if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    
    if (manager.recvfromAck(buf, &len, &from))
    {
      Serial.print("Got data from Node Station #");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      rxFlag = true; //Set rxFlag to true
      
      // Send a reply back to the originator client
      Serial.println("Sending a Reply to Node");
      if (!manager.sendtoWait(rx_reply, sizeof(rx_reply), from))
      {
          Serial.println("sendtoWait failed");
          //rxFlag = false; //Set rxFlag to false if sendtoWait fails
      }
    }
  }
}
