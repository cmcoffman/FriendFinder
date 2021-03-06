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

  // Display a simple splash screen
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

  GPS.startup();

  ffIMU.startup(false);
  Serial.println("... // Startup..[COMPLETE!]");
}

unsigned long GPS_previousMillis;

void loop() {
  GPS.update(true);
  GPS.print();
  messenger.check();
  messenger.update(false, GPS);
  ffIMU.update(false);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextFont(6);
  tft.setTextSize(1);
  tft.setCursor(1, 1);
  tft.println(millis());
  tft.setTextColor(TFT_GREEN);
  tft.setTextFont(4);

  messenger.send(true, 255);


  delay(800);
  
}
