#include "FriendFinder.h"

uint32_t timer1 = millis();
uint32_t timer2 = millis();

void setup() {
  initSerial();
  initNeopixels();
  initIMU();
  // waitForMag();
  initRadio();
  initGPS();
  colorDotWipe(BLUE, 20);
  // while (!checkMag()) colorDotWipe(YELLOW, 20);
}

void loop() {
  checkRadio(false);
   checkGPS(false);

  if (millis() - timer1 > 10000) {
    timer1 = millis();
    printGPS2();
    printPacket(inpacket);
 int distanceMeters = HaverSineFixed(GPS.latitude_fixed, GPS.longitude_fixed, inpacket.latitude_fixed, inpacket.longitude_fixed);
  //Serial.print("Distance (m): "); Serial.printn(distanceMeters);
  float d = dumbDistance(GPS.latitude_fixed, GPS.longitude_fixed, inpacket.latitude_fixed, inpacket.longitude_fixed);
  Serial.print("Distance (m): "); Serial.println(d, 6);
  }
}
