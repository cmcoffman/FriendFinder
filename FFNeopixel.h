#include <Adafruit_NeoPixel.h>

// Builtin LED's
#define LED 13

// Neopixel Stuff
#define LEDPIN 5  // Pin Neopixel Ring is on
#define NUMPIXELS 24 // Number of Pixels
#define BRIGHTNESS 1
#define TOPPIXEL 15

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

uint32_t RED = strip.Color(255, 0, 0);
uint32_t GREEN = strip.Color(0, 255, 0);
uint32_t BLUE = strip.Color(0, 0, 255);
uint32_t YELLOW = strip.Color(255, 255, 0);
uint32_t WHITE = strip.Color(255, 255, 255);
uint32_t GREY = strip.Color(64, 64, 64);
uint32_t OFF = strip.Color(0, 0, 0);
uint32_t YELLOWGREEN = strip.Color(128, 255, 0);
uint32_t PURPLE = strip.Color(255, 0, 255);



void colorDot(int pixel, uint32_t color) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (i == pixel) {
      strip.setPixelColor(i, color);
    } else {
      strip.setPixelColor(i, OFF);
    }
    strip.show();
  }
}

void colorDotBuff(int pixel, uint32_t color) {

      strip.setPixelColor(pixel, color);
}

void colorFill(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}

void colorFillBuff(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
}

void Blink(byte PIN, byte wait, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(wait);
    digitalWrite(PIN, LOW);
    delay(wait);
  }
}



void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void colorDotWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.setPixelColor(i - 1, OFF);
    strip.show();
    delay(wait);
  }
  strip.setPixelColor(strip.numPixels() - 1, OFF);
  strip.show();
}

void colorMeter(uint32_t c, int amount) {
  amount = map(amount, 0, 100, 23, 0);
  for (int i = 0; i < 24; i++) {
    if (i <= amount) {
    strip.setPixelColor(i, c);
    } else {
    strip.setPixelColor(i, OFF);
    }
  }

  strip.show();
  //Serial.print("Amount: ");
  //Serial.println(amount);

}

void colorMeterBuff(uint32_t c, int amount) {
  amount = map(amount, 0, 100, 0, 23);
  for (int i = 0; i < 24; i++) {
    if (i <= amount) {
    strip.setPixelColor(i, c);
    } else {
    strip.setPixelColor(i, OFF);
    }
  }

  //Serial.print("Amount: ");
  //Serial.println(amount);

}

void initNeopixels() {
  Serial.print("Neopixels");
  colorFill(OFF);
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show();
  Serial.println("...[OK]");
}
