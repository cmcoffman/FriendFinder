#include <Adafruit_NeoPixel.h>
#include "FriendFinder.h"

// Set Ring size
#define SMALL_NEORING_SIZE 24
// Strip is connected to Arduino Pin 1
#define STRIP_PIN 5
NeoDisplay ring = NeoDisplay(SMALL_NEORING_SIZE, STRIP_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(500);  // delay a bit when we start so we can open arduino serial
               // monitor window
  Serial.println("Starting NeoEffects Test");
  ring.begin();
  ring.clearStrip();
  ring.show();
}

void loop() {
  delay(100);
  if (millis() < 10000) {
    // ring.fillStrip(ring.randomColor());
    ring.colorDotWipe(ring.Blue, 50);
    ring.colorWipe(ring.randomColor(), 50);
    ring.colorDot(1, ring.Red);
  } else {
    ring.clearStrip();
    ring.show();
  }
}
