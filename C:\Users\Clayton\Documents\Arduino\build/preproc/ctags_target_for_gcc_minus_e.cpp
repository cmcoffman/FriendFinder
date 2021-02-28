# 1 "/Users/claytoncoffman/Documents/Arduino/FriendFinder/FriendFinder.ino"
// Are you using the PCB?
//#define FF_PCB

# 5 "/Users/claytoncoffman/Documents/Arduino/FriendFinder/FriendFinder.ino" 2
# 6 "/Users/claytoncoffman/Documents/Arduino/FriendFinder/FriendFinder.ino" 2
# 7 "/Users/claytoncoffman/Documents/Arduino/FriendFinder/FriendFinder.ino" 2
# 8 "/Users/claytoncoffman/Documents/Arduino/FriendFinder/FriendFinder.ino" 2
# 9 "/Users/claytoncoffman/Documents/Arduino/FriendFinder/FriendFinder.ino" 2

// void drawCompassWait(CRGB color, int top = TOPPIXEL);
// void drawCompass(CRGB color, int top = TOPPIXEL);

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// ffNeoRing ring = ffNeoRing(NUMPIXELS, NEOPIXEL_RING_PIN, NEO_GRB +
// NEO_KHZ800);
ffGPS GPS = ffGPS(&Serial2);
ffRadio radio(33, 21);
ffMessenger messenger(radio, 4);
ffIMU ffIMU;

int myAddy;

CRGB friendColors[5];



CRGB leds[24];
void setup() {
  Serial.begin(115200);

  // Red LED
  pinMode(13, 0x02);

  // Radio Enable Pin
  pinMode(4, 0x02);

  // Button Pin
  //pinMode(15, INPUT);
  pinMode(15, 0x05);

  digitalWrite(4, 0x1);
  //delay(5000);

  while (!Serial && millis() < 5000);


myAddy = getAddress();
Serial.print("Addy - ");
Serial.println(getAddress());

// CRGB friendColors[0] = CRGB::Red;
// CRGB friendColors[1] = CRGB::Yellow;
// CRGB friendColors[2] = CRGB::Green;
// CRGB friendColors[3] = CRGB::Blue;
// CRGB friendColors[4] = CRGB::Purple;





// radio.printRegisters();


// ring.begin();
// ring.setBrightness(60);
// ring.clearStrip();
// ring.show();
// ring.flash();

FastLED.addLeds<WS2811, A0, GRB>(leds, 24)
    .setCorrection(TypicalLEDStrip);
FastLED.setBrightness(96);
display.begin(0x02 /*|< Gen. display voltage from 3.3V*/, 0x3C);
display.clearDisplay();
display.display();

GPS.startup();

ffIMU.startup(false);

// MESSENGER MUST START FIRST!!!

messenger.startup();
//messenger.setTimeout(400);
//messenger.setRetries(0);
radio.setFrequency(915.0);
radio.setTxPower(23, false);

messenger.setThisAddress(myAddy);

Serial.print("Messenger Addy - ");
Serial.println(messenger.thisAddress());
// radio.startup();
//radio.printRegisters();


flash();
// radio.printRegisters();
//printData(true);
messenger.myAddy = getAddress();
colorWipe(getColor(1));
colorWipe(getColor(2));
colorWipe(getColor(3));
colorWipe(getColor(4));
delay(1000);
//colorWipe(getColor(myAddy));
Serial.println("-- Startup Complete --");
//flashColor(CRGB::Orange);
}

// Conveinient timer construct (at end of loop)
uint32_t timer1 = millis();
uint32_t timer2 = millis();
uint32_t timer3 = millis();
uint32_t timer4 = millis();
float dist2;

CRGB redWorm[24];
CRGB yellowWorm[24];
CRGB greenWorm[24];
CRGB blueWorm[24];

int toppix = 0;
int dotpix;

int orientRing(int heading, int top = 10) {
    // Get degrees per pixel
    int degPerPixel = 360 / 24;
    // Calculate pixel for heading
    int pixelOut = -((heading / degPerPixel) - top);
    pixelOut = (pixelOut + 24) % 24;
    dotpix = pixelOut;
    return (pixelOut);
}

void clearRing() {
  for (uint16_t i = 0; i < 24; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}


void clearRingWait() {
  for (uint16_t i = 0; i < 24; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
}

void colorDot(int pixel, CRGB color) {
  for (uint16_t i = 0; i < 24; i++) {
    if (i == pixel) {
      leds[i] = color;
    } else {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
  }
}

void colorWipe(CRGB color) {
  for (uint16_t i = 0; i < 24; i++) {
      leds[i] = color;
    FastLED.show();
    delay(10);
  }
  clearRing();
}

void colorDotWait(int pixel, CRGB color) {
  for (uint16_t i = 0; i < 24; i++) {
    if (i == pixel) {
      leds[i] = color;
    }
  }
}

void drawCompassWait(CRGB color, int top = 10) {
  int pixel = orientRing(ffIMU.event.orientation.x, top);
  colorDotWait(pixel, color);
}

void drawCompass(CRGB color, int top = 10) {
    int pixel = orientRing(ffIMU.event.orientation.x, top);
  colorDotWait(pixel, color);
  FastLED.show();
}

void flash() {
  for (int j = 255; j > 0; j--) {
    for (uint16_t i = 0; i < 24; i++) {
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

void flashColor(CRGB color) {
  clearRing();
  for (int j = 255; j > 0; j--) {
    for (uint16_t i = 0; i < 24; i++) {
      leds[i] = color;
    }
    FastLED.show();
    for (int i = 0; i < 10; i++) {
      leds[i].fadeLightBy( 256/10 );
      FastLED.show();
      delay(10);
    }
    for (int i = 0; i < 10; i++) {
      leds[i].fadeToBlackBy( 256/10 );
      FastLED.show();
      delay(10);
    }
    clearRing();
  }
  // delay(1);
  // ffNeoRing::clearStrip();
  // ffNeoRing::show();
  // show();
}


void blink() {

    for (uint16_t i = 0; i < 24; i++) {
      leds[i] = CRGB(255, 0, 255);
    }
    FastLED.show();

    for (uint16_t i = 0; i < 24; i++) {
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
  for (int i = 0; i < 24; i++) {
    worm[i] = CRGB::Black;
  }

  // Set Colors
  for (int i = center - radius; i <= center + radius; i++) {
    int j;
    j = i % (24);
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



// void printData(bool setup) {
//   if (setup) {
//     Serial.println("my.lat, my.long, their.lat, their.long, their.distance, their.bearing");
//   } else {
//     Serial.print(messenger.friend_locs[MY_ADDRESS].latitude, 9);
//     Serial.print(",");
//     Serial.print(messenger.friend_locs[MY_ADDRESS].longitude, 9);
//     Serial.print(",");
//     Serial.print(messenger.friend_locs[FRIEND_ADDRESS].latitude, 9);
//     Serial.print(",");
//     Serial.print(messenger.friend_locs[FRIEND_ADDRESS].longitude, 9);
//     Serial.print(",");
//     Serial.print(messenger.friend_locs[FRIEND_ADDRESS].distance_meters);
//     Serial.print(",");
//     Serial.println(messenger.friend_locs[FRIEND_ADDRESS].bearing);
//   }
// }

String getMacAddress() {
 uint8_t baseMac[6];
 // Get MAC address for WiFi station
 esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
 char baseMacChr[18] = {0};
 sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  Serial.print("My MAC Address: ");
  Serial.println(String(baseMacChr));
 return String(baseMacChr);
}

int getAddress() {
    // String macRed = "80:7D:3A:F0:E2:E4";
    // String macYellow = "80:7D:3A:F0:E2:E2";
    // String macGreen = "80:7D:3A:F0:E2:E1";
    // String macBlue = "80:7D:3A:F0:E2:E4";

    if (getMacAddress() == "80:7D:3A:F0:E2:E3") {
        Serial.print("Friend Address: ");
        Serial.print(0);
        Serial.println(" (Red)");
        return 0;
    }
    if (getMacAddress() == "80:7D:3A:F0:E2:E2") {
        Serial.print("Friend Address: ");
        Serial.print(1);
        Serial.println(" (Yellow)");
        return 1;
    }
    if (getMacAddress() == "80:7D:3A:BC:D3:A4") {
        Serial.print("Friend Address: ");
        Serial.print(2);
        Serial.println(" (Green)");
        return 2;
    }
    if (getMacAddress() == "80:7D:3A:F0:E2:E4" /* done*/) {
        Serial.print("Friend Address: ");
        Serial.print(3);
        Serial.println(" (Blue)");
        return 3;
    }
}

CRGB getColor(int addy) {
      if (addy == 0) {
        return CRGB::Red;
    }
      if (addy == 1) {
        return CRGB::Yellow;
    }
      if (addy == 2) {
        return CRGB::Green;
    }
    if (addy == 3) {
        return CRGB::Blue;
    }
    if (addy == 4) {
        return CRGB::Purple;
    }
}

int buttonState = 0;
void loop() {
  GPS.update(false);
  messenger.check();

if (digitalRead(15) == 0x0) {

     // typical debounce time
     delay(20);

     // Lets check again if still low
if (digitalRead(15) == 0x0) {
     {
     buttonState = (buttonState + 1) % 6;
  // Stay here if the thing we need to do is would be done before the user can let go of the button. 
  // So we dont re-trigger thinking its another press.
      while(digitalRead(15) == 0x0);
     }
    }
}


  messenger.update(false, GPS);
  ffIMU.update(false);
  clearRingWait();
//Serial.println(buttonState);
  // drawCompassWait(CRGB::Red);
  // colorDotWait(orientRing(ffIMU.event.orientation.x - messenger.friend_locs[FRIEND_ADDRESS].bearing), CRGB::Green);

  if (buttonState == 0) {
    //Serial.println(buttonState); 
  makeWorm(redWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[0].bearing), 3, CRGB::Red);
  makeWorm(yellowWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[1].bearing), 3, CRGB::Yellow);
  makeWorm(greenWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[2].bearing), 3, CRGB::Green);
  makeWorm(blueWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[3].bearing), 3, CRGB::Blue);


  for (int i = 0; i < 24; i++) {
    leds[i] = redWorm[i] + yellowWorm[i] + greenWorm[i] + blueWorm[i];
  }
  FastLED.show();
}

 if (buttonState == 1) {

  makeWorm(redWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[0].bearing), 3, CRGB::Red);
  makeWorm(yellowWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[1].bearing), 3, CRGB::Yellow);
  makeWorm(greenWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[2].bearing), 3, CRGB::Green);
  makeWorm(blueWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[3].bearing), 3, CRGB::Blue);

  clearRingWait();
  for (int i = 0; i < 24; i++) {
    leds[i] = redWorm[i];
  }
  FastLED.show();
}

if (buttonState == 2) {

makeWorm(redWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[0].bearing), 3, CRGB::Red);
  makeWorm(yellowWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[1].bearing), 3, CRGB::Yellow);
  makeWorm(greenWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[2].bearing), 3, CRGB::Green);
  makeWorm(blueWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[3].bearing), 3, CRGB::Blue);
  clearRingWait();
  for (int i = 0; i < 24; i++) {
    leds[i] = yellowWorm[i];
  }
  FastLED.show();
}
if (buttonState == 3) {
makeWorm(redWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[0].bearing), 3, CRGB::Red);
  makeWorm(yellowWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[1].bearing), 3, CRGB::Yellow);
  makeWorm(greenWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[2].bearing), 3, CRGB::Green);
  makeWorm(blueWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[3].bearing), 3, CRGB::Blue);

  clearRingWait();
  for (int i = 0; i < 24; i++) {
    leds[i] = greenWorm[i];
  }
  FastLED.show();
}

if (buttonState == 4) {
makeWorm(redWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[0].bearing), 3, CRGB::Red);
  makeWorm(yellowWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[1].bearing), 3, CRGB::Yellow);
  makeWorm(greenWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[2].bearing), 3, CRGB::Green);
  makeWorm(blueWorm, orientRing(ffIMU.event.orientation.x - messenger.friend_locs[3].bearing), 3, CRGB::Blue);

  clearRingWait();
  for (int i = 0; i < 24; i++) {
    leds[i] = blueWorm[i];
  }
  FastLED.show();
}
if (buttonState == 5) {

  clearRingWait();
  drawCompass(CRGB::Pink);
  FastLED.show();
}

  // Periodic Action
  if (timer1 > millis()) timer1 = millis(); // fix rollover
  if (millis() - timer1 > 10000) {
    timer1 = millis(); // reset the timer
    digitalWrite(13, 0x1);

    timer4 = millis();
    // messenger.send(true, 0);
    // delay(500);
    // messenger.send(true, 1);
    // delay(500);
    // messenger.send(true, 2);
    // delay(500);
    // messenger.send(true, 3);
    // delay(500);

    messenger.send(true, 255);

    Serial.print("transmission time: ");
    Serial.println(millis() - timer4);

     digitalWrite(13, 0x0);
  }

if (timer2 > millis()) timer2 = millis(); // fix rollover
  if (millis() - timer2 > 6000) {
    timer2 = millis(); // reset the timer
    Serial.println("----------");
    Serial.println("My GPS:");
    GPS.print();
    Serial.println("");
    Serial.println("My Message:");
    messenger.printPacket(messenger.outPacket);
    Serial.println("");
    Serial.println("Friend inPacket:");
    messenger.printPacket(messenger.inPacket);
    Serial.println("");
    Serial.println("Friend DB entry:");
    messenger.printPacket(messenger.friend_msgs[2]);
    Serial.println("");
    Serial.print("My heading: ");
    Serial.println(ffIMU.event.orientation.x);
    Serial.print("Bearing to friend: ");
    Serial.println(messenger.friend_locs[2].bearing);
    Serial.print("Distance to friend: ");
    Serial.println(messenger.friend_locs[2].distance_meters);


    //printData(false);
  }

  if (timer3 > millis()) timer3 = millis(); // fix rollover
  if (millis() - timer3 > 60000) {
    timer3 = millis(); // reset the timer
    //printData(true);
  }
}
