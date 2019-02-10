#include <RH_RF95.h>
// Radio Stuff
// for Prototype Board
#define RFM95_CS 6
#define RFM95_RST 11
#define RFM95_INT 10


/* for Feather32u4 RFM9x
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 7
*/

/* for feather m0 RFM9x
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 3
*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Message Recieve Buffer
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];



void initRadio() {
  Serial.print("Radio...");
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("[FAIL]");
    while (1);
  }


  //  while (!rf95.init()) {
  //    Serial.print(".");
  //  }
  //  while (!rf95.available()) {
  //    Serial.print(".");
  //  }

  Serial.println("[OK]");
   Serial.print("Setting radio frequency...");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("[FAIL]");
    while (1);
  }
  Serial.println("[OK]");

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

bool checkRadio(bool verbose = true) {
  //Serial.println("Radio: check");
  if (rf95.available()) {
    if (verbose) Serial.println("Radio: available");
    // Should be a message for us now
    //uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv((uint8_t *)&buf, &len)) {
      if (verbose) Serial.println("Radio: Message received!");
      //colorFill(WHITE);
      digitalWrite(LED, HIGH);
      delay(10);
      digitalWrite(LED, LOW);
      //colorFill(OFF);
      if (verbose) {
        RH_RF95::printBuffer("Received: ", buf, len);
        //Serial.print("Got: ");
        //Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);

        // if (newDataPacket.fixquality != 0) {
        // Serial.print("GPS Fix: ");
        // Serial.print(newDataPacket.latitudeDegrees, 8);
        // Serial.print(", ");
        // Serial.println(newDataPacket.longitudeDegrees, 8);
        // } else {
        //   Serial.println("Friend has no GPS fix.");
        // }
      }
    return true;
    }
    else
    {
      if (verbose) Serial.println("Radio: No message received.");
      return false;
    }
  }
}
