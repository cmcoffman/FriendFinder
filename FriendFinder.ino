#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>
#include "FriendFinder.h"

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

ffNeoRing ring = ffNeoRing(NUMPIXELS, NEOPIXEL_RING_PIN, NEO_GRB + NEO_KHZ800);
ffGPS GPS = ffGPS(&GPSSerial);
ffRadio radio(RFM95_CS, RFM95_INT);
ffMessenger messenger(radio, MY_ADDRESS);
ffIMU ffIMU;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  ring.begin();
  ring.setBrightness(60);
  ring.clearStrip();
  ring.show();
  ring.flash();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Address 0x3C for 128x32
  display.clearDisplay();
  display.display();

  GPS.startup();
  radio.startup();
  ffIMU.startup();
}

// Conveinient timer construct (at end of loop)
uint32_t timer1 = millis();
uint32_t timer2 = millis();
float dist2;

void loop() {
  GPS.update(false);
  messenger.check(false);
  messenger.update(false, GPS);

  // if millis() or timer wraps around, we'll just reset it
  if (timer1 > millis()) timer1 = millis();
  // approximately every 2 seconds or so, print out the current stats

    messenger.send(true, FRIEND_ADDRESS);
    // }
  }

  if (timer2 > millis()) timer2 = millis();
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer2 > 500) {
    timer2 = millis();  // reset the timer
    // text display tests
    dist2 = messenger.haversine(
        messenger.outPacket.latitude_fixed, messenger.outPacket.longitude_fixed,
        messenger.inPacket.latitude_fixed, messenger.inPacket.longitude_fixed);
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("D: ");
    display.print(dist2, 0);
    display.println(" m");
    display.print("T: ");
    unsigned long age = messenger.time_since_last_msg / 1000;
    display.print(age);
    display.print(" s");
    display.setCursor(0, 0);
    display.display();  // actually display all of the above
  }
}
