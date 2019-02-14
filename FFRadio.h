#include <RHReliableDatagram.h>
#include <RH_RF95.h>
// Radio Stuff
// for Prototype Board
#define RFM95_CS 6
#define RFM95_RST 11
#define RFM95_INT 10

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

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

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(rf95, SERVER_ADDRESS);

void initRadio() {
  Serial.println("Bootup...");
 // pinMode(LEDPIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);

  // manual reset
  Serial.println("Reset Radio...");
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) Serial.println("RF95 Init - FAIL");
  if (rf95.init()) Serial.println("RF95 Init - OK");
  if (!manager.init()) Serial.println("Datagram Manager Init - FAIL");
  if (manager.init()) Serial.println("Datagram Manager Init - OK");

  Serial.print("Set Radio Freq...");
  if (!rf95.setFrequency(RF95_FREQ)) Serial.println("[FAIL]");
  if (rf95.setFrequency(RF95_FREQ)) Serial.println("[Succeed]");
  Serial.println("Set Radio power...");
  rf95.setTxPower(23, false);
  Serial.println("-End Radio Init-");
  }

// Data Packet
struct dataPacket {
  uint32_t latitude_fixed;      // 4 bytes
  uint32_t longitude_fixed;     // 4 bytes
  uint8_t fixquality;                // 1 byte
  uint8_t satellites;                // 1 byte
  uint16_t HDOP;                    // 4 bytes
};

struct dataPacket outpacket;
struct dataPacket inpacket;


//uint8_t data[] = "Hello World!";
uint8_t data[sizeof(outpacket)];
// Message Recieve Buffer
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

// Reply Message

void printPacket(dataPacket packet) {
    Serial.print("FR Lat FIX: "); Serial.println(packet.latitude_fixed);
    Serial.print("FR Lon FIX: "); Serial.println(packet.longitude_fixed);
    //Serial.print("Fix Qual: "); Serial.println(packet.fixquality);
    //Serial.print("Sats: "); Serial.println(packet.satellites);
    //Serial.print("HDOP: "); Serial.println(packet.HDOP);
    // Convert back to float for printing.
    float HDOP;
    HDOP = (float)packet.HDOP / 100;
    Serial.print("HDOP: "); Serial.println(HDOP);


}

bool checkRadio(bool verbose = true) {
 if (manager.available()) {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from)) {
        // copy message to inpacket
        memcpy(&inpacket, buf, sizeof(inpacket));
      if (verbose) {
        Serial.print("got request from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        for (int i = 0; i < (RH_RF95_MAX_MESSAGE_LEN - 1); i++){
          Serial.print(buf[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }

      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from))
        Serial.println("sendtoWait failed");
    }
  }
}
