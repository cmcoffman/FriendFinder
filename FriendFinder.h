#ifndef _FRIENDFINDER_
#define _FRIENDFINDER_

#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

class ffNeoRing : public Adafruit_NeoPixel {
 private:
 public:
  // Constructor
  ffNeoRing(uint16_t n, uint8_t p = 6, uint8_t t = NEO_GRB + NEO_KHZ800);

  // some simple functions to effect whole strip
  void clearStrip();
  void fillStrip(uint32_t c);
  void show(void);
  void colorDotWipe(uint32_t c, uint8_t wait);
  void colorWipe(uint32_t c, uint8_t wait);
  void colorDot(int pixel, uint32_t color);

  // helper functions dealing with Adafruit_NeoPixel::Color (32 bit color)
  static uint32_t randomColor(void) {
    return Adafruit_NeoPixel::Color(random(0, 255), random(0, 255),
                                    random(0, 255));
  }
  static uint32_t randomWheelColor(void);
  static uint32_t colorWheel(
      byte WheelPos); /*!< colorWheel defines 255 colors of full intensity */
  static const uint32_t Red;
  static const uint32_t Green;
  static const uint32_t Blue;
  static const uint32_t Yellow;
  static const uint32_t White;
  static const uint32_t Grey;
  static const uint32_t Off;
  static const uint32_t YellowGreen;
  static const uint32_t Purple;
};

class ffGPS : public Adafruit_GPS {
 private:
 public:
  // Constructor
  ffGPS(HardwareSerial* ser);

  // GPS Functions
  void startup(bool verbose = true);
  void print(bool verbose = true);
  void update(bool verbose = true);

  // GPS Data
  // uint8_t hour, minute, seconds, year, month, day;
  //   uint16_t milliseconds;
  //   // Floating point latitude and longitude value in degrees.
  //   float latitude, longitude;
  //   // Fixed point latitude and longitude value with degrees stored in units
  //   of 1/100000 degrees,
  //   // and minutes stored in units of 1/100000 degrees.  See pull #13 for
  //   more details:
  //   //   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
  //   int32_t latitude_fixed, longitude_fixed;
  //   float latitudeDegrees, longitudeDegrees;
  //   float geoidheight, altitude;
  //   float speed, angle, magvariation, HDOP;
  //   char lat, lon, mag;
  //   boolean fix;
  //   uint8_t fixquality, satellites;
};

// Radio Stuff
// for Prototype Board
// #define RFM95_CS 6
// #define RFM95_RST 11
// #define RFM95_INT 10

// #define CLIENT_ADDRESS 1
// #define SERVER_ADDRESS 2

// for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7

/* for feather m0 RFM9x
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 3
*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Data Packet
struct dataPacket {
  uint32_t latitude_fixed;   // 4 bytes
  uint32_t longitude_fixed;  // 4 bytes
  uint8_t fixquality;        // 1 byte
  uint8_t satellites;        // 1 byte
  uint16_t HDOP;             // 2 bytes
};

class ffRadio : public RH_RF95 {
 private:
 public:
  // Constructor
  ffRadio(uint8_t csPin, uint8_t intPin);
  void startup(bool verbose = true);
  // void print(bool verbose);
  // void update(bool verbose);
};

class ffMessenger : public RHReliableDatagram {
 private:
 public:
  // Constructor
  ffMessenger(RHGenericDriver& driver, uint8_t thisAddress);
  void startup(bool verbose = true);
  void printPacket(dataPacket packet);
  void check(bool verbose = true);
  void update(bool verbose, ffGPS myGPS);
  void send(bool verbose, uint8_t to);

  // Ingoing and Outgoing Datapackets
  dataPacket inPacket;
  dataPacket outPacket;

  // Message Recieve Buffer
  uint8_t inBuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t outBuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(inBuf);
};

#endif  // Close Library
