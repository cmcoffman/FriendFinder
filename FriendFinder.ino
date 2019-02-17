#include <Adafruit_NeoPixel.h>
#include "FriendFinder.h"

// Set Ring size
#define SMALL_NEORING_SIZE 24
// Strip is connected to Arduino Pin 1
#define STRIP_PIN 5
NeoStrip strip1 = NeoStrip(SMALL_NEORING_SIZE, STRIP_PIN, NEO_GRB + NEO_KHZ800);



void setup() {
  Serial.begin(115200);
  delay(500); // delay a bit when we start so we can open arduino serial monitor window
  Serial.println("Starting NeoEffects Test");
  strip1.begin();
  strip1.clearStrip();
  strip1.show();
}

void loop() {
  strip1.fillStrip(strip1.randomColor());
  delay(100);
  if (millis() > 10000) {
    strip1.clearStrip();
  }
  strip1.show();
}
