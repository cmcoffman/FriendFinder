#ifndef _FRIENDFINDER_
#define _FRIENDFINDER_

#include <Adafruit_NeoPixel.h>
#include <Adafruit_GPS.h>

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

  //helper functions dealing with Adafruit_NeoPixel::Color (32 bit color)
  static uint32_t randomColor(void) {
    return Adafruit_NeoPixel::Color(random(0, 255), random(0, 255),
                                    random(0, 255));
  }
  static uint32_t randomWheelColor(void);
  static uint32_t colorWheel(byte WheelPos); /*!< colorWheel defines 255 colors of full intensity */
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
  ffGPS(HardwareSerial *ser);

  // GPS Functions
void startup(bool verbose = true);
void print(bool verbose = true);
void update(bool verbose);

// GPS Data
// uint8_t hour, minute, seconds, year, month, day;
//   uint16_t milliseconds;
//   // Floating point latitude and longitude value in degrees.
//   float latitude, longitude;
//   // Fixed point latitude and longitude value with degrees stored in units of 1/100000 degrees,
//   // and minutes stored in units of 1/100000 degrees.  See pull #13 for more details:
//   //   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
//   int32_t latitude_fixed, longitude_fixed;
//   float latitudeDegrees, longitudeDegrees;
//   float geoidheight, altitude;
//   float speed, angle, magvariation, HDOP;
//   char lat, lon, mag;
//   boolean fix;
//   uint8_t fixquality, satellites;
};


#endif  // Close Library
