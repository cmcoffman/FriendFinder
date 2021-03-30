#include "FriendFinder.h"

ffGPS GPS = ffGPS(&GPSSerial);
ffRadio radio(RFM95_CS, RFM95_INT);
ffMessenger messenger(radio, 2);
ffIMU ffIMU;
ffEntanglement entanglement(GPS, messenger, ffIMU);
#ifdef FFMK2
ffDisplay ffDisplay(&entanglement);
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

  ffDisplay.startup(true);

  // Display a simple splash screen
  ffDisplay.fillScreen(TFT_BLACK);
  ffDisplay.setTextSize(2);
  ffDisplay.setTextColor(TFT_WHITE);
  ffDisplay.setCursor(40, 5);
  ffDisplay.print(F("Friend"));
  delay(700);
  ffDisplay.setTextColor(TFT_PINK);
  ffDisplay.print(F("Finder"));
  delay(700);
  ffDisplay.setCursor(35, 25);
  ffDisplay.println(F("MK2"));
  delay(1000);
  ffDisplay.fillScreen(TFT_BLACK);

  // messenger.startup(true);

  // messenger.setTimeout(400);
  // messenger.setRetries(0);
  radio.setFrequency(RF95_FREQ);
  radio.setTxPower(23, false);

  // messenger.setThisAddress(3);

  //GPS.startup();

  ffIMU.startup(false);
  Serial.println("... // Startup..[COMPLETE!]");

  ffDisplay.setPage(SCREEN_OFF);
  delay(500);
  //ffDisplay.setPage(2);
}

unsigned long GPS_previousMillis;

void loop() {
  GPS.update(true);
  //GPS.print();
  // messenger.check();
  // messenger.update(false, GPS);
  ffIMU.update(false);
  


  // messenger.send(true, 255);
  entanglement.entangle(ffIMU);
  //entanglement.entangle(ffGPS);
  //entanglement.entangle(ffIMU);
  //entanglement.printSelfStatus();
  ffDisplay.setPage(STATUS_SCREEN);
  ffDisplay.drawPage();

  delay(80);
  
}
