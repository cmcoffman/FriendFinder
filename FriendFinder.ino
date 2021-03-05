#include "FriendFinder.h"

ffGPS GPS = ffGPS(&GPSSerial);
ffRadio radio(RFM95_CS, RFM95_INT);
ffMessenger messenger(radio, 2);
ffIMU ffIMU;
#ifdef FFMK2
ffDisplay tft;
#endif

void setup() {
  delay(3000);
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB
  }

  Serial.println(">>> FF STARTUP ./././ ... ");
  delay(1000);
  Serial.println("no messenger.startup()");
  Serial.println("Display Only?");
  radio.reset(true);

  tft.startup(true);

  //Display a simple splash screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(40, 5);
  tft.print(F("Friend"));
  delay(700);
  tft.setTextColor(TFT_PINK);
  tft.print(F("Finder"));
  delay(700);
  tft.setCursor(35, 25);
  tft.println(F("MK2"));
  delay(1000);
  tft.fillScreen(TFT_BLACK);

  messenger.startup(true);

  messenger.setTimeout(400);
  messenger.setRetries(0);
  radio.setFrequency(RF95_FREQ);
  radio.setTxPower(23, false);

  messenger.setThisAddress(3);

  //GPS.startup();

  ffIMU.startup(false);
  Serial.println("... // Startup..[COMPLETE!]");
}

void loop() {

  // GPS.update(false);
  // messenger.check();
  // messenger.update(false, GPS);
  // ffIMU.update(false);
  Serial.print(" milliseconds since startup: ");
  Serial.println(millis());
    //Display a simple splash screen
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(40, 5);
  tft.print(F("Friend"));
  delay(700);
  tft.setTextColor(TFT_GREEN);
  tft.print(F("Finder"));
  delay(700);
  tft.setCursor(35, 25);
  tft.println(F("MK2"));
  delay(1000);
  tft.fillScreen(TFT_BLACK);
  // tft.setCursor(0, 0);
  // tft.print("Message ");
  // delay(300);
  // tft.print(". ");
  // delay(300);
  // tft.print(". ");
  // delay(300);
  // tft.print(". ");
  // delay(300);
  messenger.send(true, 255);
  // tft.println(".sent!");


  delay(1800);
}


void screen_splash() {
  byte font = 1;
  tft.setTextFont(font);
  tft.setTextSize(3);
  // tft.fillScreen(TFT_BLACK);
  int padding =
      tft.textWidth("999", font);  // get the width of the text in pixels
  tft.setTextColor(TFT_GREEN, TFT_BLUE);
  tft.setTextPadding(padding);
  // tft.setTextColor(TFT_BLACK, TFT_RED);
  tft.setCursor(0, 0, 2);
  // Set the font colour to be white with a black background, set text size multiplier to 1
  tft.setTextColor(TFT_WHITE,TFT_BLACK);  tft.setTextSize(1);
  // We can now plot text on screen using the "print" class
  tft.println("Hello World!");
  tft.drawCentreString("FriendFinder", TFT_HEIGHT / 2, TFT_WIDTH / 2, 1);
  // tft.drawRightString("FriendFinder", 150, 50, 1);
}