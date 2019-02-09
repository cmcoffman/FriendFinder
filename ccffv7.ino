#include "FriendFinder.h"

uint32_t timer = millis();

void setup() {
  initSerial();
  initNeopixels();
  initIMU();
  //waitForMag();
  initRadio();
  initGPS();
  colorDotWipe(BLUE, 20);
  //while (!checkMag()) colorDotWipe(YELLOW, 20);
  
}



bool magGood = false;
void loop() {
  colorFillBuff(OFF);
  if (!checkMag() && magGood == false) {
    colorDotBuff(23, GREY);
    colorDotBuff(4, GREY);
  } else {
    magGood = true;
    
  }
  checkRadio(false);
  checkGPS(false);
  //printGPS();
  checkIMU();
    
  //drawDistance();


  
  int distance = HaverSine(GPS.latitudeDegrees, GPS.longitudeDegrees, newDataPacket.latitudeDegrees, newDataPacket.longitudeDegrees);
  if (distance > 5000) {
    colorDotBuff(0, RED);
    colorDotBuff(1, RED);
    colorDotBuff(2, RED);
    colorDotBuff(3, RED);
  } else {
    colorDotBuff(0, OFF);
    colorDotBuff(1, OFF);
    colorDotBuff(2, OFF);
    colorDotBuff(3, OFF);
    
  }
  

  drawCompass();
   drawFriend();
 
  
  
  if (millis() - timer >2000) {
     timer = millis();
     printFriend();
     drawCompass(true);
     drawFriend(true);
  
  } 
  strip.show();
}
