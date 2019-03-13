#include "FriendFinder.h"



ffNeoRing ring = ffNeoRing(NUMPIXELS, NEOPIXEL_RING_PIN, NEO_GRB + NEO_KHZ800);
ffGPS GPS = ffGPS(&GPSSerial);
ffRadio radio(RFM95_CS, RFM95_INT);
ffMessenger messenger(radio, MY_ADDRESS);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  delay(500);
  Serial.println("Starting NeoEffects Test");
   ring.begin();
   ring.clearStrip();
   ring.show();

  GPS.startup(true);
  radio.startup(true);
  messenger.startup(true);
}

// Conveinient timer construct (at end of loop)
uint32_t timer = millis();

void loop() {
  delay(100);
  GPS.update(true);
  messenger.check(true);
  messenger.update(false, GPS);
  if (millis() < 10000) {
    ring.colorDotWipe(ring.Blue, 20);
    ring.colorWipe(ring.randomColor(), 20);
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
    Serial.println("*Beacon heartbeat*");

    //messenger.update(true, GPS);
    Serial.println("IN Packet:");
    messenger.printPacket(messenger.inPacket);
    Serial.println("OUT Packet:");
    messenger.printPacket(messenger.outPacket);
      // if (GPS.fixquality > B0) {
        messenger.send(true, 255);
  // }
}
}
