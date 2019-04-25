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
  //GPS.update(false);
  //messenger.check(false);
  //messenger.update(false, GPS);
  ffIMU.update(false);

  //  displayData();

  int heading = ffIMU.event.orientation.x;
  int pixel = ring.orientRing(heading);
  ring.colorDot(pixel, ffNeoRing::Blue);
  //delay(100);

  // // Periodic Action
  // if (timer2 > millis()) timer2 = millis(); // fix rollover
  // if (millis() - timer2 > 2000) {
  //   timer2 = millis();  // reset the timer

  // }
}

void displayData() {
    dist2 = messenger.haversine(
        messenger.outPacket.latitude_fixed, messenger.outPacket.longitude_fixed,
        messenger.inPacket.latitude_fixed, messenger.inPacket.longitude_fixed);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("D: ");
    display.print(dist2, 0);
    display.print(" m ");
    display.print("T: ");
    unsigned long age = messenger.time_since_last_msg / 1000;
    display.print(age);
    display.print(" s ");
    display.println();
    //display.println();
    display.print("Sys:");
    display.print(ffIMU.system, DEC);

    display.print(" G:");
    display.print(ffIMU.gyro, DEC);

    display.print(" A:");
    display.print(ffIMU.accel, DEC);

    display.print(" M:");
    display.print(ffIMU.mag, DEC);
display.println();
    display.println();


    display.print("X: ");
    display.print(ffIMU.event.orientation.x, 0);
    display.print(" Y: ");
    display.print(ffIMU.event.orientation.y, 0);
    display.print(" Z: ");
    display.print(ffIMU.event.orientation.z, 0);






    display.setCursor(0, 0);
    display.display();  // actually display all of the above
}
