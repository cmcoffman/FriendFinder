#include "FriendFinder.h"

ffGPS GPS = ffGPS(&GPSSerial);
ffRadio radio(RFM95_CS, RFM95_INT);
ffMessenger messenger(radio, 4);
ffIMU ffIMU;
//#ifdef FFMK2
ffDisplay tft;
//#endif

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);

  GPS.startup();

  ffIMU.startup(false);
  tft.startup(true);
  // MESSENGER MUST START FIRST!!!

  messenger.startup();
  radio.setFrequency(RF95_FREQ);
  radio.setTxPower(23, false);


  Serial.println("-- Startup Complete --");
}

// Conveinient timer construct (at end of loop)
uint32_t timer1 = millis();
uint32_t timer2 = millis();
uint32_t timer3 = millis();
uint32_t timer4 = millis();
float dist2;

void loop() {
  GPS.update(false);
  messenger.check();
  messenger.update(false, GPS);
  ffIMU.update(false);
  
  // Periodic Action
  if (timer1 > millis()) timer1 = millis();  // fix rollover
  if (millis() - timer1 > 10000) {
    timer1 = millis();  // reset the timer
    digitalWrite(13, HIGH);

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

    digitalWrite(13, LOW);
  }

  if (timer2 > millis()) timer2 = millis();  // fix rollover
  if (millis() - timer2 > 6000) {
    timer2 = millis();  // reset the timer
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

    // printData(false);
  }

  if (timer3 > millis()) timer3 = millis();  // fix rollover
  if (millis() - timer3 > 60000) {
    timer3 = millis();  // reset the timer
    // printData(true);
  }
}
