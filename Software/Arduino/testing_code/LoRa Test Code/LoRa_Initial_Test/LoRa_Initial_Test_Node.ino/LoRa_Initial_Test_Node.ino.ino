// LoRa Node Test Code  

// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h> // https://github.com/PaulStoffregen/RadioHead

// First 3 here are boards w/radio BUILT-IN. Boards using FeatherWing follow.

// Feather M0:
#define RFM95_CS   10
#define RFM95_RST  11
#define RFM95_INT  12

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------
#define NODE_STATION   true    // Set if node is just to transmit messages by LoRa

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
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// ----------------------------------------------------------------------------
// Datagram Set-up (Manages which Node and Base Station are communicating)
// ----------------------------------------------------------------------------
#if NODE_STATION
RHReliableDatagram manager(rf95, NODE_ADDRESS);  // (driver, address)Class to manage message delivery and receipt, using the driver declared above
#endif

#if BASE_STATION
RHReliableDatagram manager(rf95, BASE_ADDRESS);  // (driver,address)Class to manage message delivery and receipt, using the driver declared above
#endif


// ----------------------------------------------------------------------------
// Unions/structures 
// ----------------------------------------------------------------------------
typedef union {
  char d[20] = "hello base";
} SBD_MO_MESSAGE;

SBD_MO_MESSAGE tx_message;        //Outgoing LoRa message 
size_t messageSize = sizeof(tx_message); // Size of data to be stored and transmitted

// ----------------------------------------------------------------------------
// Global Variables 
// ----------------------------------------------------------------------------
int16_t                 packetnum;                              // LoRa radio packet number
uint8_t                 buf[RH_RF95_MAX_MESSAGE_LEN];           // LoRa radio buffer
uint8_t                 len               = sizeof(buf);        // LoRa radio buffer length
volatile bool           loraRxFlag        = false;              // LoRa message received flag

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

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

//int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop() {
  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
  Serial.println("Attempting to Transmit Message to Base via LoRa..."); // Send a message to rf95_server
  uint8_t from;
  // Send a message to manager_server
  if (manager.sendtoWait((uint8_t *)&tx_message, sizeof(tx_message), BASE_ADDRESS))
  {
    // Now wait for a reply from the server
    uint8_t len = sizeof(buf);
 
    if (manager.recvfromAckTimeout(buf, &len, 4000, &from))
    {
      Serial.print("got reply from base station #:");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);
      loraRxFlag = true;
    }
    else
    {
      Serial.println("No reply, is the base station running?");
    }
  }
  else
    Serial.println("sendtoWait failed");

  Serial.print("RSSI = "); Serial.println(rf95.lastRssi()); //received signal strenght indicator- minimum -91

}
