#include <Adafruit_NeoPixel.h>
#include "FriendFinder.h"

// Set Ring size
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 5
ffNeoRing ring = ffNeoRing(NUMPIXELS, NEOPIXEL_RING_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting NeoEffects Test");
  ring.begin();
  ring.clearStrip();
  ring.show();
}

void loop() {
  delay(100);
  if (millis() < 10000) {
    ring.colorDotWipe(ring.Blue, 50);
    ring.colorWipe(ring.randomColor(), 50);
    ring.colorDot(1, ring.Red);
  } else {
    ring.clearStrip();
    ring.show();
  }
}
