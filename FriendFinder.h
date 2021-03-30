#ifndef _FRIENDFINDER_
#define _FRIENDFINDER_

#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Button2.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <utility/imumaths.h>
// Font files are stored in SPIFFS, so load the linbrary
#include <FS.h>

#include "FFConfig.h"
//#include "esp_system.h"

class ffGPS : public Adafruit_GPS {
 private:
 public:
  // Constructor
  ffGPS(HardwareSerial* ser);

  // GPS Functions
  void startup(bool verbose = true);
  void print(bool verbose = true);
  void update(bool verbose = true);
  /*/
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
/*/
};

// Data Packet
struct dataPacket {
  int32_t latitude_fixed;   // 4 bytes
  int32_t longitude_fixed;  // 4 bytes
  uint8_t fixquality;       // 1 byte
  uint8_t satellites;       // 1 byte
  uint16_t HDOP;            // 2 bytes
};

// Friends Loc and Distance
struct friendDB {
  float latitude;
  float longitude;
  float distance_meters;
  float bearing;
  int age_seconds;
  int quality;
};

// Status
struct ffStatus {
  String macAddress;
  uint8_t ffAddress; // This is the address for the radio/datagram too
  uint16_t ffColor; // For displays/neopixels
  uint8_t messageCounter = 0; // total messages recieved from friend
  uint8_t fix_quality;
  float latitude;
  float longitude;
  float orientation_x;
  float orientation_y;
  float orientation_z;
  float battery_V;
};


class ffRadio : public RH_RF95 {
 private:
 public:
  // Constructor
  ffRadio(uint8_t csPin, uint8_t intPin);
  void startup(bool verbose = true);
  void reset(bool verbose = true);
  // void print(bool verbose);
  // void update(bool verbose);
};

class ffMessenger : public RHReliableDatagram {
 private:
 public:
  // Constructor
  ffMessenger(RHGenericDriver& driver, uint8_t thisAddress);

  // Methods
  void startup(bool verbose = true);
  void printPacket(dataPacket packet);
  void check(bool verbose = true);
  void update(bool verbose, ffGPS myGPS);
  void send(bool verbose, uint8_t to);

  uint32_t haversine(float lat1, float lon1, float lat2, float lon2);

  int bearing(float lat1, float lon1, float lat2, float lon2);

  // Ingoing and Outgoing Datapackets
  dataPacket inPacket;
  dataPacket outPacket;

  // Friend Message Array (holds ten friends as is)
  dataPacket friend_msgs[10];

  // Latest message form
  uint8_t lastFrom;

  // Time since last Message
  unsigned long time_of_last_msg;
  unsigned long time_of_last_check;
  unsigned long time_since_last_msg;

  // Summarized Friend Location Data
  friendDB friend_locs[10];

  // Message Recieve Buffer
  uint8_t inBuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t outBuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(inBuf);

  // My address
  uint8_t myAddy;
};

class ffIMU : public Adafruit_BNO055 {
 private:
 public:
  // Constructor
  ffIMU();

  // Methods
  void startup(bool verbose = true);
  void update(bool verbose = true);
  void displaySensorDetails();
  void displaySensorStatus();
  void displayCalStatus();

  // Event
  sensors_event_t event;

  // Calibration Status
  uint8_t system, gyro, accel, mag;
};

class ffEntanglement {
 private:
 public:
  // Constructor
  ffEntanglement(ffGPS myGPS, ffMessenger myMessenger, ffIMU myIMU);

  // Methods
  void update(ffGPS myGPS, ffMessenger myMessenger, ffIMU myIMU);
  void entangle(ffStatus aFriend);
  void entangle(ffGPS myGPS);
  void entangle(ffMessenger myMessenger);
  void entangle(ffIMU myIMU);
  
  void printSelfStatus();
  String getMacAddress();
  

  // Data
  ffStatus selfStatus;
  ffStatus friendStatus[10];

  // Preloaded Data
  String knownMacAddresses[5] = {"80:7D:3A:F0:E2:E3", "80:7D:3A:F0:E2:E2",
                                "80:7D:3A:BC:D3:A4", "24:62:AB:CB:17:00",
                                "80:7D:3A:F0:E2:E4"};
  uint16_t friendColors[5] = {TFT_RED, TFT_YELLOW, TFT_GREEN, TFT_PURPLE, TFT_BLUE};

};

class ffDisplay : public TFT_eSPI {
// Fonts
#define NOTOSANSBOLD15 "NotoSansBold15"
#define NOTOSANSBOLD36 "NotoSansBold36"
#define BARCODE20 "LibreBarcode20"
#define BARCODE15 "LibreBarcode15"
#define BARCODE36 "LibreBarcode36"
#define BARCODE72 "LibreBarcode72"
#define LCARS28 "LCARS28"
#define LCARS20 "LCARS20"

// Colors
#define LCARS_ORANGE 0xFCC0
#define LCARS_PURPLE 0xCB33
#define LCARS_PINK 0xCCD9
#define LCARS_YELLOW 0xFE6C
#define LCARS_RED 0x8902
#define LCARS_BLUE 0x9CDF
#define LCARS_PALEBLUE 0x9CD9


 private:
  #define SCREEN_OFF 0
  #define SPLASH_SCREEN 1
  #define STATUS_SCREEN 2
  int page;
  bool page_change = true;
  TFT_eSPI tft;
  // Screen Buffers
   TFT_eSprite screenBuffer = TFT_eSprite(&tft); 
   TFT_eSprite statusScreen = TFT_eSprite(&tft); 

  // Sprites
  TFT_eSprite sprite_IMU = TFT_eSprite(&tft); 

  // Font math
  int fontHeight;
  int lineWidth;
  int characterWidth;
  
  void draw_sprite_IMU(); // static elements
  void update_sprite_IMU(); // dynamic elements

  // Entanglement Pointer
  ffEntanglement * _myEntanglment;
 public:
  // Constructor
  ffDisplay(ffEntanglement *myEntanglement);



  // Methods
  void startup(bool verbose = true);
  void setPage(int new_page);
  void getPage(int current_page);
  void drawPage();
  void screen_off();
  void screen_splash();
  void screen_status();
  


  

};

#endif  // Close Library
