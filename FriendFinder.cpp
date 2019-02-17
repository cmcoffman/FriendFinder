// NeoDisplay.cpp
// a wrapper/dervied class of Adafruit_NeoPixel to add some functionality
// most everything passes on to the parent class

#include "FriendFinder.h"
#include <Adafruit_NeoPixel.h>

const uint32_t NeoDisplay::Red = Adafruit_NeoPixel::Color(255, 0, 0);
const uint32_t NeoDisplay::Green = Adafruit_NeoPixel::Color(0, 255, 0);
const uint32_t NeoDisplay::Blue = Adafruit_NeoPixel::Color(0, 0, 255);
const uint32_t NeoDisplay::Yellow = Adafruit_NeoPixel::Color(255, 255, 0);
const uint32_t NeoDisplay::White = Adafruit_NeoPixel::Color(255, 255, 255);
const uint32_t NeoDisplay::Grey = Adafruit_NeoPixel::Color(64, 64, 64);
const uint32_t NeoDisplay::Off = Adafruit_NeoPixel::Color(0, 0, 0);
const uint32_t NeoDisplay::YellowGreen = Adafruit_NeoPixel::Color(128, 255, 0);
const uint32_t NeoDisplay::Purple = Adafruit_NeoPixel::Color(255, 0, 255);


// wrapper on Adafruit_NeoPixel constructor
NeoDisplay::NeoDisplay(uint16_t n, uint8_t p, uint8_t t)
    : Adafruit_NeoPixel(n, p, t) {}

// Color Dot Runs around ring
void NeoDisplay::colorDotWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < numPixels(); i++) {
    setPixelColor(i, c);
    setPixelColor(i - 1, Adafruit_NeoPixel::Color(0, 0, 0));
    show();
    delay(wait);
  }
  setPixelColor(numPixels() - 1, Adafruit_NeoPixel::Color(0, 0, 0));
  show();
}

// Fill Whole Ring with color
void NeoDisplay::colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < numPixels(); i++) {
    setPixelColor(i, c);
    show();
    delay(wait);
  }
}

void NeoDisplay::colorDot(int pixel, uint32_t color) {
  for (uint16_t i = 0; i < numPixels(); i++) {
    if (i == pixel) {
      setPixelColor(i, color);
    } else {
      setPixelColor(i, Off);
    }
    show();
  }
}

// overload the base class show to check if stripChanged
void NeoDisplay::show(void) {
    Adafruit_NeoPixel::show();
    }

void NeoDisplay::clearStrip() {
  for (int i = 0; i < numPixels(); i++) setPixelColor(i, 0);
}

void NeoDisplay::fillStrip(uint32_t c) {
  for (int i = 0; i < numPixels(); i++) setPixelColor(i, c);
}

uint32_t NeoDisplay::randomWheelColor(void) {
    return colorWheel(random(0, 255));
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t NeoDisplay::colorWheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return Adafruit_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return Adafruit_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return Adafruit_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}
