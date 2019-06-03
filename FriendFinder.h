#ifndef _FRIENDFINDER_
#define _FRIENDFINDER_

#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <FastLED.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <utility/imumaths.h>

#include "FFConfig.h"

// FastLED Ring Suff
#ifdef ARDUINO_FEATHER_ESP32

//#define CLK_PIN   4
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS 24

#define BRIGHTNESS 96
#define FRAMES_PER_SECOND 120
#endif

#ifdef ARDUINO_SAMD_FEATHER_M0_EXPRESS
// FastLED
#define DATA_PIN 5
//#define CLK_PIN   4
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS 24

#define BRIGHTNESS 96
#define FRAMES_PER_SECOND 120
#endif

class ffNeoRing : public Adafruit_NeoPixel {
 private:
 public:
  // Constructor
  ffNeoRing(uint16_t n, uint8_t p = 6, uint8_t t = NEO_GRB + NEO_KHZ800);

  // some simple functions to effect whole strip
  void clearStrip();
  void fillStrip(uint32_t c);
  void show(void);
  void colorDotWipe(uint32_t c, uint16_t wait);
  void colorWipe(uint32_t c, uint16_t wait);
  void colorDot(int pixel, uint32_t color);
  void flash();
  int orientRing(int heading);

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

// Data Packet
struct dataPacket {
  int32_t latitude_fixed;   // 4 bytes
  int32_t longitude_fixed;  // 4 bytes
  uint8_t fixquality;        // 1 byte
  uint8_t satellites;        // 1 byte
  uint16_t HDOP;             // 2 bytes
};

// Friends Loc and Distance
struct friendDB {
  float latitude;
  float longitude;
  float distance_meters;
  float bearing;
  int age_seconds;
  int quality;
  CRGB color;
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

  // Methods
  void startup(bool verbose = true);
  void printPacket(dataPacket packet);
  void check(bool verbose = true);
  void update(bool verbose, ffGPS myGPS);
  void send(bool verbose, uint8_t to);

  uint32_t haversine(float lat1, float lon1, float lat2, float lon2);

  int bearing(float lat1, float lon1, float lat2,
                           float lon2);

      // Ingoing and Outgoing Datapackets
  dataPacket inPacket;
  dataPacket outPacket;

  // Friend Message Array (holds ten friends as is)
  dataPacket friend_msgs[10];

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

#endif  // Close Library
