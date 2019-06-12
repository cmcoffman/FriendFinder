// Are you using the PCB?
//#define FF_PCB

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <Wire.h>
#include "FriendFinder.h"

// void drawCompassWait(CRGB color, int top = TOPPIXEL);
// void drawCompass(CRGB color, int top = TOPPIXEL);

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// ffNeoRing ring = ffNeoRing(NUMPIXELS, NEOPIXEL_RING_PIN, NEO_GRB +
// NEO_KHZ800);
ffGPS GPS = ffGPS(&GPSSerial);
ffRadio radio(RFM95_CS, RFM95_INT);
ffMessenger messenger(radio, MY_ADDRESS);
ffIMU ffIMU;

CRGB leds[NUM_LEDS];
void setup() {
  Serial.begin(115200);

  // Red LED
  pinMode(13, OUTPUT);

  // Radio Enable Pin
  pinMode(4, OUTPUT);

  // Button Pin
  //pinMode(15, INPUT);
  pinMode(15, INPUT_PULLUP);

  digitalWrite(4, HIGH);
  //delay(5000);

  while (!Serial && millis() < 5000);



 // radio.printRegisters();


  // ring.begin();
  // ring.setBrightness(60);
  // ring.clearStrip();
  // ring.show();
  // ring.flash();

  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS)
      .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);


  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  GPS.startup();

  ffIMU.startup();

  // MESSENGER MUST START FIRST!!!

      messenger.startup();
      //messenger.setTimeout(400);
      //messenger.setRetries(0);
            radio.setFrequency(RF95_FREQ);
    radio.setTxPower(23, false);

   // radio.startup();
   //radio.printRegisters();


  flash();

   // radio.printRegisters();
   //printData(true);
   Serial.println("-- Startup Complete --");
}

// Conveinient timer construct (at end of loop)
uint32_t timer1 = millis();
uint32_t timer2 = millis();
uint32_t timer3 = millis();
float dist2;

CRGB worm1[NUM_LEDS];
CRGB worm2[NUM_LEDS];
CRGB worm3[NUM_LEDS];
int toppix = 0;
int dotpix;

int orientRing(int heading, int top = TOPPIXEL) {
    // Get degrees per pixel
    int degPerPixel = 360 / NUMPIXELS;
    // Calculate pixel for heading
    int pixelOut = -((heading / degPerPixel) - top);
    pixelOut = (pixelOut + NUMPIXELS) % NUMPIXELS;
    dotpix = pixelOut;
    return (pixelOut);
}

void clearRing() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

void clearRingWait() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
}

void colorDot(int pixel, CRGB color) {
  for (uint16_t i = 0; i < NUMPIXELS; i++) {
    if (i == pixel) {
      leds[i] = color;
    } else {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
  }
}

void colorDotWait(int pixel, CRGB color) {
  for (uint16_t i = 0; i < NUMPIXELS; i++) {
    if (i == pixel) {
      leds[i] = color;
    }
  }
}

void drawCompassWait(CRGB color, int top = TOPPIXEL) {
  int pixel = orientRing(ffIMU.event.orientation.x, top);
  colorDotWait(pixel, color);
}

void drawCompass(CRGB color, int top = TOPPIXEL) {
    int pixel = orientRing(ffIMU.event.orientation.x, top);
  colorDotWait(pixel, color);
  FastLED.show();
}

void flash() {
  for (int j = 255; j > 0; j--) {
    for (uint16_t i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(j / 1.1, j / 1.1, j / 1.1);
    }
    // delayMicroseconds(50);
    FastLED.show();
  }
  // delay(1);
  // ffNeoRing::clearStrip();
  // ffNeoRing::show();
  // show();
}

void blink() {

    for (uint16_t i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255, 0, 255);
    }
    FastLED.show();

    for (uint16_t i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
  // delay(1);
  // ffNeoRing::clearStrip();
  // ffNeoRing::show();
  // show();
}

// this is that gradient worm
void makeWorm(CRGB *worm, int center, int radius, CRGB color) {
  // Compute lenth of the worm
  int length = (radius * 2) + 1;

  // Clear Worm
  for (int i = 0; i < NUM_LEDS; i++) {
    worm[i] = CRGB::Black;
  }

  // Set Colors
  for (int i = center - radius; i <= center + radius; i++) {
    int j;
    j = i % (NUM_LEDS);
    j = abs(j);
    if (i == center) {
      worm[j] = color;
    } else {
      CRGB color2 = color;

      worm[j] = color;
      int pos = i - center;
      pos = abs(pos) + 1;
      color2.nscale8(255 / pow(pos, 3));
      worm[j] = color2;
    }
  }
}

void displayData() {
  // dist2 = messenger.haversine(
  //     messenger.outPacket.latitude_fixed, messenger.outPacket.longitude_fixed,
  //     messenger.inPacket.latitude_fixed, messenger.inPacket.longitude_fixed);
  dist2 = messenger.friend_locs[FRIEND_ADDRESS].distance_meters;
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.setCursor(0, 0);

  display.print("D:");
  if (dist2 > 8000) display.print("???");
  else display.print(dist2, 0);
  display.print("m ");

  display.print("T:");
  unsigned long age = messenger.time_since_last_msg / 1000;
  display.print(age);
  display.print("s ");

  display.print("H:");

  display.print(messenger.friend_locs[FRIEND_ADDRESS].bearing);


  display.println();
  display.print("Sys:");
  display.print(ffIMU.system, DEC);

  display.print(" G:");
  display.print(ffIMU.gyro, DEC);

  display.print(" A:");
  display.print(ffIMU.accel, DEC);

  display.print(" M:");
  display.print(ffIMU.mag, DEC);


  display.println();
  display.print("TOP:");
  display.print(toppix);
  display.print(" DOT:");
  display.print(dotpix);

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

void printData(bool setup) {
  if (setup) {
    Serial.println("my.lat, my.long, their.lat, their.long, their.distance, their.bearing");
  } else {
    Serial.print(messenger.friend_locs[MY_ADDRESS].latitude, 9);
    Serial.print(",");
    Serial.print(messenger.friend_locs[MY_ADDRESS].longitude, 9);
    Serial.print(",");
    Serial.print(messenger.friend_locs[FRIEND_ADDRESS].latitude, 9);
    Serial.print(",");
    Serial.print(messenger.friend_locs[FRIEND_ADDRESS].longitude, 9);
    Serial.print(",");
    Serial.print(messenger.friend_locs[FRIEND_ADDRESS].distance_meters);
    Serial.print(",");
    Serial.println(messenger.friend_locs[FRIEND_ADDRESS].bearing);
  }
}
void loop() {
  GPS.update(false);
  messenger.check(false);

if (digitalRead(15) == LOW) {
  Serial.println("Button LOW");
  blink();
}

// if (digitalRead(15) == HIGH) {
//   Serial.println("Button HIGH");
// //  flash();
// }

  // Exact Coords of Apmt
  // 361373800,867853733
  // 361373130,867853250
  // 36.137313, -86.785325

  // back porch Coords
  // 361372990 867852690

  // mailbox coords
  // 361375000 867851560


// // Fake data for testing
//   messenger.friend_msgs[FRIEND_ADDRESS].latitude_fixed = 361372990;
//   messenger.friend_msgs[FRIEND_ADDRESS].longitude_fixed = 867852690;
//   messenger.friend_msgs[FRIEND_ADDRESS].fixquality = 1;
//   messenger.outPacket.latitude_fixed = 361375000;
//   messenger.outPacket.longitude_fixed = 867851560;


  messenger.update(false, GPS);
  ffIMU.update(false);



  clearRingWait();

 // colorDotWait(0, CRGB::Red);
  drawCompassWait(CRGB::Red);
  colorDotWait(orientRing(ffIMU.event.orientation.x - messenger.friend_locs[FRIEND_ADDRESS].bearing), CRGB::Green);


  //  makeWorm(worm1, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[FRIEND_ADDRESS].heading_degrees), 3, CRGB::Green);

  displayData();

  // for (int i = 0; i < NUM_LEDS; i++) {

  //    leds[i] = worm2[i] + worm1[i] + worm3[i] + leds[i];
  //   //leds[i] = worm1[i];
  // }
  FastLED.show();


 // int heading = ffIMU.event.orientation.x;
  // int pixel = orientRing(heading);
  // colorDot(pixel, CRGB(0, 0, 255));
  // colorDot(TOPPIXEL, CRGB(255, 0, 0));
  // ring.colorDot(pixel, ffNeoRing::Blue);
  // delay(100);

  // makeWorm(worm2, orientRing(ffIMU.event.orientation.x) + 6 , 3, CRGB::Red);
  // makeWorm(worm3, orientRing(ffIMU.event.orientation.x) - 6, 3,
  // CRGB::Purple);
 // for (int i = 0; i < NUM_LEDS; i++) {

     //leds[i] = worm2[i] + worm1[i] + worm3[i] ;
  //   //leds[i] = worm1[i];
  // }

  // FastLED.show();
  // int pixel = orientRing(ffIMU.event.orientation.x);

  // Periodic Action
  if (timer1 > millis()) timer1 = millis();  // fix rollover
  if (millis() - timer1 > 5000) {
    timer1 = millis();  // reset the timer
    digitalWrite(13, HIGH);
    // GPS.update(true);
    // messenger.check(true);

    messenger.send(true, FRIEND_ADDRESS);

     digitalWrite(13, LOW);
  }

if (timer2 > millis()) timer2 = millis();  // fix rollover
  if (millis() - timer2 > 6000) {
    timer2 = millis();  // reset the timer
    // Serial.println("----------");
    // Serial.println("My GPS:");
    // GPS.print();
    // Serial.println("");
    // Serial.println("My Message:");
    // messenger.printPacket(messenger.outPacket);
    // Serial.println("");
    // Serial.println("Friend inPacket:");
    // messenger.printPacket(messenger.inPacket);
    // Serial.println("");
    // Serial.println("Friend DB entry:");
    // messenger.printPacket(messenger.friend_msgs[FRIEND_ADDRESS]);
    // Serial.println("");
    // Serial.print("My heading: ");
    // Serial.println(ffIMU.event.orientation.x);
    // Serial.print("Bearing to friend: ");
    // Serial.println(messenger.friend_locs[FRIEND_ADDRESS].heading_degrees);
    // Serial.print("Distance to friend: ");
    // Serial.println(messenger.friend_locs[FRIEND_ADDRESS].distance_meters);


    printData(false);
  }

  if (timer3 > millis()) timer3 = millis();  // fix rollover
  if (millis() - timer3 > 60000) {
    timer3 = millis();  // reset the timer
    //printData(true);
  }
}
