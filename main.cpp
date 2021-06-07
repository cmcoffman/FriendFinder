#include <Arduino.h>

#include "FriendFinder.h"

ffGPS GPS = ffGPS(&GPSSerial);
ffRadio radio(RFM95_CS, RFM95_INT);
ffMessenger messenger(radio, 2);
ffIMU ffIMU;
ffEntanglement entanglement(GPS, messenger, ffIMU);
#ifdef FFMK2
ffDisplay ffDisplay(&entanglement);
#endif
#ifdef FFMK3
ffDisplay ffDisplay(&entanglement);
#endif

// OTA Stuff
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char *ssid = "***REMOVED***";
const char *password = "***REMOVED***";

void setup() {
  //delay(1000);
  Serial.begin(115200);
  Serial.println(">>> FF STARTUP ./././ ... ");

#pragma region Wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("FFProto");

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else  // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
      })
      .onEnd([]() { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      })
      .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
          Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
          Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
          Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
          Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
          Serial.println("End Failed");
      });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#pragma endregion

  // delay(1000);
  Serial.println("no messenger.startup()");
  Serial.println("Display Only?");
  // radio.reset(true);

  ffDisplay.startup(true);

  // Display a simple splash screen
  // ffDisplay.fillScreen(TFT_BLACK);
  // ffDisplay.setTextSize(2);
  // ffDisplay.setTextColor(LCARS_ORANGE);
  // ffDisplay.setCursor(40, 5);
  // ffDisplay.print(F("Friend"));
  // delay(250);
  // ffDisplay.print(F("Finder"));
  // delay(250);
  // ffDisplay.setCursor(35, 25);
  // ffDisplay.println(F("MK3"));
  // delay(750);
  // ffDisplay.fillScreen(TFT_BLACK);

  // messenger.startup(true);

  // messenger.setTimeout(400);
  // messenger.setRetries(0);
  // radio.setFrequency(RF95_FREQ);
  // radio.setTxPower(23, false);

  // messenger.setThisAddress(3);

  //ffDisplay.print(F("Friend"));
//  ffDisplay.terminalScreen.setCursor(40, 5);
  //ffDisplay.terminalScreen.setTextSize(2);
  ffDisplay.terminalScreen.print(F("GPS ///"));
  if (GPS.startup()) {
    ffDisplay.terminalScreen.println(F("[OK]"));
  } else {
    ffDisplay.terminalScreen.println(F("[FAIL]"));
  }
  ffDisplay.terminalScreen.pushSprite(60, 48);

// ffDisplay.print(F("GPS ///"));
//   if (GPS.startup()) {
//     ffDisplay.println(F("[OK]"));
//   } else {
//     ffDisplay.println(F("[FAIL]"));
//   }

// ffDisplay.terminalScreen.scroll(0, + ffDisplay.terminalScreen.fontHeight()); 
// delay(1000);
// ffDisplay.terminalScreen.pushSprite(120, 48);

// ffDisplay.terminalScreen.scroll(0, + ffDisplay.terminalScreen.fontHeight()); 
// delay(1000);
// ffDisplay.terminalScreen.pushSprite(120, 48);

Serial.print("Font height:");
Serial.println(ffDisplay.terminalScreen.fontHeight());

  // ffDisplay.setTextSize(2);
  //   ffDisplay.setCursor(40, 5);
  // ffDisplay.print(F("Friend"));

  ffIMU.startup(false);
  Serial.println("... // Startup..[COMPLETE!]");

  // ffDisplay.setPage(SCREEN_OFF);
  // delay(500);
  // ffDisplay.setPage(2);
  // ffDisplay.screen_off();
}

unsigned long GPS_previousMillis;

void loop() {
  ArduinoOTA.handle();
  GPS.update(true);
  //GPS.print();
  // messenger.check();
  // messenger.update(false, GPS);
  ffIMU.update(false);

  // messenger.send(true, 255);
  entanglement.entangle(ffIMU);
  // entanglement.entangle(ffGPS);
  // entanglement.entangle(ffIMU);
  // entanglement.printSelfStatus();
  // ffDisplay.setPage(STATUS_SCREEN);
  // ffDisplay.drawPage();

  delay(80);
}
