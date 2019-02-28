#include "FriendFinder.h"


// Neopixel Ring Stuff
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 5
ffNeoRing ring = ffNeoRing(NUMPIXELS, NEOPIXEL_RING_PIN, NEO_GRB + NEO_KHZ800);

// GPS Stuff
#define GPSSerial Serial1
ffGPS GPS = ffGPS(&GPSSerial);

// Radio Stuff
// for Prototype Board
// #define RFM95_CS 6
// #define RFM95_RST 11
// #define RFM95_INT 10


// for Feather32u4 RFM9x
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 7


/* for feather m0 RFM9x
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 3
*/

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

ffRadio radio(RFM95_CS, RFM95_INT);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(500);
  Serial.println("Starting NeoEffects Test");
  ring.begin();
  ring.clearStrip();
  ring.show();

  GPS.startup();
  radio.startup();

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
