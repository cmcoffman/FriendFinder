#ifndef _FRIENDFINDER_
#define _FRIENDFINDER_

#include <Adafruit_NeoPixel.h>

class NeoStrip : public Adafruit_NeoPixel {
 private:
 public:
  NeoStrip(uint16_t n, uint8_t p = 6, uint8_t t = NEO_GRB + NEO_KHZ800);

  // some simple functions to effect whole strip
  void clearStrip();
  void fillStrip(uint32_t c);
  void show(void);

  //helper functions dealing with Adafruit_NeoPixel::Color (32 bit color)
  static uint32_t randomColor(void) {
    return Adafruit_NeoPixel::Color(random(0, 255), random(0, 255),
                                    random(0, 255));
  }
  static uint32_t randomWheelColor(void);
  static const uint32_t White; /*!< quick common reference for full white color */
  static uint32_t colorWheel(byte WheelPos); /*!< colorWheel defines 255 colors of full intensity */
};

#endif  // Close Library
