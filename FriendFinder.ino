#include "FriendFinder.h"


// Neopixel Ring Stuff
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 5
ffNeoRing ring = ffNeoRing(NUMPIXELS, NEOPIXEL_RING_PIN, NEO_GRB + NEO_KHZ800);

// GPS Stuff
#define GPSSerial Serial1
ffGPS GPS = ffGPS(&GPSSerial);

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting NeoEffects Test");
  ring.begin();
  ring.clearStrip();
  ring.show();
  //GPS.begin(9600);
  GPS.startup();
}

// Conveinient timer construct (at end of loop)
uint32_t timer = millis();

void loop() {
  //delay(100);
  GPS.update(true);
  if (millis() < 10000) {
    ring.colorDotWipe(ring.Blue, 50);
    ring.colorWipe(ring.randomColor(), 50);
    ring.colorDot(1, ring.Red);
  } else {
    ring.clearStrip();
    ring.show();
  }

    // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 5000) {
    timer = millis();  // reset the timer
    Serial.println("*heartbeat*");
    Serial.println(GPS.latitude_fixed);


  }
}
