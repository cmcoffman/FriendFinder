#include "FriendFinder.h"
#include <math.h>

#pragma region ffGPS
ffGPS::ffGPS(HardwareSerial* ser) : Adafruit_GPS(ser) {}

bool ffGPS::startup(bool verbose) {
  bool output;
  
  if (verbose) Serial.print("GPS Startup....");

#ifdef FFMK2
  GPSSerial.begin(9600, SERIAL_8N1, GPSRX, GPSTX);

  if(!GPSSerial.begin(9600, SERIAL_8N1, GPSRX, GPSTX)) {
    Serial.println("GPS - FAIL");
  } else {
    Serial.println("GPS - OK");
  }

#else
  if(Adafruit_GPS::begin(9600)) {
    output = true;
    if (verbose) Serial.println("OK");
  } else {
    output = false;
    if (verbose) Serial.println("FAIL");
  };
#endif

  // turn on only GPRMC sentence
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // turn on GPRMC and GGA
  Adafruit_GPS::sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GGA only
  // Adafruit_GPS::sendCommand(
  //   "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
  // turn on ALL THE DATA
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  // turn off output
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);

  // How often position is echoed via NMEA
  // o actually speed up the position fix you must also
  // send one of the position fix rate commands below
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ);  // 1 per
  // 10s
  Adafruit_GPS::sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);  // 1 per 5s
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // 1 per second
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);    // 2 per second
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);    // 5 per second
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   // 10 per second

  // How often the GPS solves it's position
  // Never tested these
  // Adafruit_GPS::sendCommand(PMTK_API_SET_FIX_CTL_100_MILLIHERTZ); // 1 per
  // 10s Adafruit_GPS::sendCommand(PMTK_API_SET_FIX_CTL_200_MILLIHERTZ); // 1
  // per 5s Adafruit_GPS::sendCommand(PMTK_API_SET_FIX_CTL_1HZ);    // 1 per
  // second Adafruit_GPS::sendCommand(PMTK_API_SET_FIX_CTL_5HZ);    // 5 per
  // second Can't fix position faster than 5 times a second!

  // Request updates on antenna status, comment out to keep quiet
  // Adafruit_GPS::sendCommand(PGCMD_ANTENNA);

  // Ask for firmware version
  // Serial.println(PMTK_Q_RELEASE);
  return output;
}
void ffGPS::update(bool verbose) {
  char c = Adafruit_GPS::read();

  if (Adafruit_GPS::newNMEAreceived()) {
    if (verbose) {
      Serial.println("********************************");
      Serial.println("***New NMEA Sentence Received***");
      Serial.println("********************************");
      Serial.println("NMEA Sentence:");
      Serial.println(Adafruit_GPS::lastNMEA());
    }

    if (!Adafruit_GPS::parse(Adafruit_GPS::lastNMEA())) {
      // this also sets the newNMEAreceived() flag to false
      if (verbose) {
        Serial.println("*Failed to Parse*");
        Serial.println("********************************");
        Serial.println("***      End of Message      ***");
        Serial.println("********************************");
      }
      return;  // we can fail to parse a sentence in which case we should just
               // wait for another
    } else {
      if (verbose) {
        ffGPS::print(verbose);
        Serial.println("********************************");
        Serial.println("***      End of Message      ***");
        Serial.println("********************************");
      }
    }
  } else {
    if (verbose) {
      // Decided not to print a message if there's nothing (spammy)
      // Serial.println("*No NMEA*");
    }
  }
}
void ffGPS::print(bool verbose) {
  Serial.println("***     Parsed GPS Data:     ***");
  if (Adafruit_GPS::fixquality == B0) {  // note use of binary "B0"
    Serial.print("Time: ");
    Serial.print(Adafruit_GPS::hour, DEC);
    Serial.print(':');
    Serial.print(Adafruit_GPS::minute, DEC);
    Serial.print(':');
    Serial.print(Adafruit_GPS::seconds, DEC);
    Serial.print('.');
    Serial.println(Adafruit_GPS::milliseconds);
    Serial.print("Fix quality: ");
    Serial.println((int)Adafruit_GPS::fixquality);
    Serial.println("(No Location Fix)");
  }

  if (Adafruit_GPS::fixquality > B0) {  // note use of binary "B0"
    Serial.println("*GPS Fix*");
    Serial.print("Time: ");
    Serial.print(Adafruit_GPS::hour, DEC);
    Serial.print(':');
    Serial.print(Adafruit_GPS::minute, DEC);
    Serial.print(':');
    Serial.print(Adafruit_GPS::seconds, DEC);
    Serial.print('.');
    Serial.println(Adafruit_GPS::milliseconds);
    Serial.print("Location: ");
    Serial.print(Adafruit_GPS::latitudeDegrees, 7);
    Serial.print(", ");
    Serial.println(Adafruit_GPS::longitudeDegrees, 7);
    Serial.print("Location (Fixed Width): ");
    Serial.print(Adafruit_GPS::latitude_fixed);
    Serial.print(", ");
    Serial.println(Adafruit_GPS::longitude_fixed);
    Serial.print("Fix Quality: ");
    Serial.println((int)Adafruit_GPS::fixquality);
    Serial.print("Satellites: ");
    Serial.println((int)Adafruit_GPS::satellites);
    Serial.print("HDOP: ");
    Serial.println(Adafruit_GPS::HDOP, 2);
    Serial.print("Altitude: ");
    Serial.println(Adafruit_GPS::altitude);
    Serial.print("Geoid Height: ");
    Serial.println(Adafruit_GPS::geoidheight);
  }
}
#pragma endregion

#pragma region ffDisplay
ffDisplay::ffDisplay(ffEntanglement *myEntanglement) : TFT_eSPI() {
  _myEntanglment = myEntanglement;
  page = 0;
  page_change = true;

   
}


void ffDisplay::startup(bool verbose) {
  // Setup Display
  pinMode(TFT_BL, OUTPUT);
  if (verbose) Serial.println("Display Startup...");
  ffDisplay::init();
  ffDisplay::fillScreen(TFT_BLACK);
  
  
  ffDisplay::fillScreen(TFT_RED);
  delay(100);
  ffDisplay::fillScreen(TFT_ORANGE);
  delay(100);
  ffDisplay::fillScreen(TFT_YELLOW);
  delay(100);
  ffDisplay::fillScreen(TFT_GREEN);
  delay(100);
  ffDisplay::fillScreen(TFT_BLUE);
  delay(100);
  ffDisplay::fillScreen(TFT_NAVY);
  delay(100);
  ffDisplay::fillScreen(TFT_VIOLET);
  delay(100);
  ffDisplay::fillScreen(TFT_BLACK);

  ffDisplay::setRotation(1);  // 1 is USB on right
                             // 2 is USB on top
                             // 3 is USB on left
                             // 4 is USB on bottom
                             // 0 - bottom; 5 - right
  // ffDisplay::setCursor(1,1);

   // // Setup Screens/Sprites
// Create a sprite for the scrolling numbers
  //terminalScreen.setColorDepth(8);
  terminalScreen.setRotation(1);
  terminalScreen.createSprite(135, 240);
  terminalScreen.fillSprite(TFT_BLUE);                                      // Fill sprite with blue
  //terminalScreen.setScrollRect(0, 0, 240, 135, TFT_RED); // here we set scroll gap fill color to blue
  //terminalScreen.setScrollRect(0, terminalScreen.fontHeight(), 240, 135); // here we set scroll gap fill color to blue
  terminalScreen.setTextColor(TFT_WHITE);                                   // White text, no background
  //terminalScreen.setTextDatum(TL_DATUM);                                    // Bottom right coordinate datum
  terminalScreen.setTextFont(2);
  
  //terminalScreen.setCursor(0, 135 - terminalScreen.fontHeight());
  terminalScreen.setCursor(0, 0);

  #define CHECK_FONTS
  #ifdef CHECK_FONTS
  if (!SPIFFS.begin())
  {
    Serial.println("SPIFFS initialisation failed!");
    while (1)
      yield(); // Stay here twiddling thumbs waiting
  }
  Serial.println("\r\nSPIFFS available!");

  // ESP32 will crash if any of the fonts are missing
  bool font_missing = false;
  if (SPIFFS.exists("/NotoSansBold15.vlw") == false)
    font_missing = true;
  if (SPIFFS.exists("/NotoSansBold36.vlw") == false)
    font_missing = true;
  if (SPIFFS.exists("/LibreBarcode20.vlw") == false)
    font_missing = true;
  if (SPIFFS.exists("/LibreBarcode15.vlw") == false)
    font_missing = true;
  if (SPIFFS.exists("/LibreBarcode36.vlw") == false)
    font_missing = true;
  if (SPIFFS.exists("/LibreBarcode72.vlw") == false)
    font_missing = true;
  if (SPIFFS.exists("/LCARS28.vlw") == false)
    font_missing = true;
  if (SPIFFS.exists("/LCARS20.vlw") == false)
    font_missing = true;

  if (font_missing)
  {
    Serial.println("\r\nFont missing in SPIFFS, did you upload it?");
    while (1)
      yield();
  }
  else
    Serial.println("\r\nFonts found OK.");
  #endif


  
  
}
void ffDisplay::setPage(int new_page) {
  if (page != new_page) {
    page = new_page;
    page_change = true;
  } else {
    page_change = false;
  }
}
void ffDisplay::drawPage() {
  if (page_change) {
    fillScreen(TFT_BLACK);
  }
  if (page != SCREEN_OFF) {
    digitalWrite(TFT_BL, HIGH);
  }

#define NUM_PAGES 4
  switch (page) {
    case SCREEN_OFF:
      ffDisplay::screen_off();
      page_change = false;
      break;
    case SPLASH_SCREEN:
      ffDisplay::screen_splash();
      page_change = false;
      break;
    case STATUS_SCREEN:
      ffDisplay::screen_status();
      page_change = false;
      break;
      // default:
      //   ffDisplay::screen_splash();
      //   page_change = false;
      //   break;
    // case TERMINAL_SCREEN:
    //   // ffDisplay::terminal();
    //   // page_change = false;
    //   // break;
  }
}
void ffDisplay::screen_off() {
  fillScreen(TFT_BLACK);
  digitalWrite(TFT_BL, LOW);
}
void ffDisplay::screen_splash() {}
void ffDisplay::screen_status() {}
void ffDisplay::draw_sprite_IMU() {
  tft.loadFont(LCARS28);
  fontHeight = tft.fontHeight();
  lineWidth = tft.textWidth("X-999.9");  
  characterWidth = tft.textWidth("X");  

  //Serial.printf("Font Height: %d px", fontHeight); Serial.println();
  //Serial.printf("Max Width: %d px", lineWidth); Serial.println();
  //Serial.printf("Sprite Width: %d px", lineWidth + 2); Serial.println();
  //Serial.printf("Sprite Height: %d px", (fontHeight * 3)+1); Serial.println();
  sprite_IMU.createSprite(57, 82);
  sprite_IMU.loadFont(LCARS28);
  sprite_IMU.setTextDatum(CR_DATUM);
  sprite_IMU.setTextColor(LCARS_ORANGE, TFT_BLACK);
  sprite_IMU.drawString("X", 0 + characterWidth, 0 + (fontHeight * 0) + fontHeight/2);
  sprite_IMU.drawString("Y", 0 + characterWidth, 0 + (fontHeight * 1) + fontHeight/2);
  sprite_IMU.drawString("Z", 0 + characterWidth, 0 + (fontHeight * 2) + fontHeight/2);
}
void ffDisplay::update_sprite_IMU() {
  tft.loadFont(LCARS28);
  fontHeight = tft.fontHeight();
  lineWidth = tft.textWidth("X-999.9");  
  characterWidth = tft.textWidth("X");  
   sprite_IMU.setTextDatum(CR_DATUM);
  int padding = sprite_IMU.textWidth("-999.9"); 
  sprite_IMU.setTextPadding(padding);
  sprite_IMU.setTextColor(LCARS_PALEBLUE, TFT_BLACK);

  
  //float x = *_myEntanglment->selfStatus.orientation_x;
  float y = 1.0;
  float z = 1.0;

  // sprite_IMU.drawFloat(*_myEntanglment->selfStatus.orientation_x, 1, 0 + sprite_IMU.textWidth("X-999.0"), 0 + (fontHeight * 0) + fontHeight/2);
  // sprite_IMU.drawFloat(y, 1, 0 + sprite_IMU.textWidth("X-999.0"), 0 + (fontHeight * 1) + fontHeight/2);
  // sprite_IMU.drawFloat(z, 1, 0 + sprite_IMU.textWidth("X-999.0"), 0 + (fontHeight * 2) + fontHeight/2);
}
#pragma endregion

#pragma region ffRadio
ffRadio::ffRadio(uint8_t csPin, uint8_t intPin) : RH_RF95(csPin, intPin) {}
void ffRadio::startup(bool verbose) {
  if (verbose) Serial.println("Radio Startup...");
#ifdef FFMK1
  // Radio Enable Pin
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  delay(100);
#endif
  // // Manual Reset
  // digitalWrite(RFM95_RST, HIGH);
  // pinMode(RFM95_RST, OUTPUT);
  // digitalWrite(RFM95_RST, HIGH);
  // delay(100);
  // digitalWrite(RFM95_RST, LOW);
  // delay(10);
  // digitalWrite(RFM95_RST, HIGH);
  // delay(10);
  // delay(1000);
  // pinMode(RFM95_RST, OUTPUT);
  // digitalWrite(RFM95_RST, HIGH);
  // delay(100);
  // digitalWrite(RFM95_RST, LOW);
  // delay(10);
  // digitalWrite(RFM95_RST, HIGH);
  // delay(10);

  bool init = RH_RF95::init();
  if (!init && verbose) Serial.println("RF95 Init - FAIL");
  if (init && verbose) Serial.println("RF95 Init - OK");

  if (verbose) Serial.print("Set Radio Freq...");
  if (!RH_RF95::setFrequency(RF95_FREQ) && verbose) Serial.println("[FAIL]");
  if (RH_RF95::setFrequency(RF95_FREQ) && verbose) Serial.println("[Succeed]");

  if (verbose) Serial.println("Set Radio power...[?]");
  RH_RF95::setTxPower(23, false);
  if (verbose) Serial.println("-End Radio Init-");
}
void ffRadio::reset(bool verbose) {
  if (verbose) Serial.print("Reset Radio...");
  // Manual Reset
  digitalWrite(RFM95_RST, HIGH);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  delay(1000);
  if (verbose) Serial.println("done!");
}

// // Messenger
// // cribbed from
// https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
// convert back to floating point
// lat1 /= 10000000;
// lon1 /= 10000000;
// lat2 /= 10000000;
// lon2 /= 10000000;
// distance between latitudes
// and longitudes
// Haversine Distance Calculator with Fixed width coords
// // from: http://forum.arduino.cc/index.php?topic=45760.msg332014#msg332014
// int ffMessenger::HaverSineFixed(uint32_t lat1, uint32_t lon1, uint32_t lat2,
//                                 uint32_t lon2) {
//   float flat1 = lat1 / 10000000;
//   float flat2 = lat2 / 10000000;
//   float flon1 = lon1 / 10000000;
//   float flon2 = lon2 / 10000000;
//
//   float ToRad = PI / 180.0;
//   float R = 63710;  // radius earth in m
//
//   float dLat = (lat2 - lat1) * ToRad;
//   float dLon = (lon2 - lon1) * ToRad;
//
//   float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * ToRad) *
//                                                 cos(lat2 * ToRad) *
//                                                 sin(dLon / 2) * sin(dLon /
//                                                 2);
//
//   float c = 2 * atan2(sqrt(a), sqrt(1 - a));
//
//   float d = ((R * c) * 8) / 1000;  // convert to meters
//   int distance = d;
//   return distance;
// }
// // Gotten from:
// https://gis.stackexchange.com/questions/252672/calculate-bearing-between-two-decimal-gps-coordinates-arduino-c?newreg=eb676d9dca8f4cc8ad10c14a3b00d423
#pragma endregion

#pragma region ffMessenger
ffMessenger::ffMessenger(RHGenericDriver& driver, uint8_t thisAddress)
    : RHReliableDatagram(driver, thisAddress) {
  lastFrom = 4;
}
void ffMessenger::startup(bool verbose) {
  if (verbose) Serial.println("Messenger Startup...");

#ifdef FFMK2
    // Setup HSPI
#define HSPI_SCK 25
#define HSPI_MISO 27
#define HSPI_MOSI 26
  SPI.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI);
  delay(1000);
#endif

  if (!RHReliableDatagram::init() && verbose)
    Serial.println("Messenger Init - FAIL");
  if (RHReliableDatagram::init() && verbose)
    Serial.println("Messenger Init - OK");
}
void ffMessenger::printPacket(dataPacket packet) {
  Serial.println("Packet Contents:");
  Serial.print("Latitude (fixed width): ");
  Serial.println(packet.latitude_fixed);
  Serial.print("Longitude (fixed width): ");
  Serial.println(packet.longitude_fixed);
  Serial.print("Fix Qual: ");
  Serial.println(packet.fixquality);
  Serial.print("Satellites: ");
  Serial.println(packet.satellites);
  // Convert HDOP back to float for printing.
  float HDOP;
  HDOP = (float)packet.HDOP / 100;
  Serial.print("HDOP: ");
  Serial.println(HDOP);
}
void ffMessenger::check(bool verbose) {
  // record time this check was made
  time_of_last_check = millis();
  if (RHReliableDatagram::available()) {
    // Wait for a message addressed to us from the client
    // uint8_t len = sizeof(buf);
    // What address is this from?
    uint8_t from;
    if (RHReliableDatagram::recvfromAck(inBuf, &len, &from)) {
      // copy message to inpacket
      memcpy(&inPacket, inBuf, sizeof(inPacket));
      memcpy(&friend_msgs[from], inBuf, sizeof(inPacket));
      // reset time since last message
      time_of_last_msg = millis();
      lastFrom = from;
      if (verbose) {
        Serial.print("got request from : ");
        Serial.print(from, DEC);
        Serial.print(": ");

        for (int i = 0; i < (RH_RF95_MAX_MESSAGE_LEN - 1); i++) {
          Serial.print(inBuf[i], HEX);
          // Serial.print(" ");
          // colorWipe(getColor(from));
        }
        Serial.print("LAtest = ");
        Serial.println(lastFrom);
        Serial.println();
      }

      // // Send a reply back to the originator client
      // if (!manager.sendtoWait(data, sizeof(data), from))
      //   Serial.println("sendtoWait failed");
      time_since_last_msg = millis() - time_of_last_msg;
      lastFrom = from;
    }
  } else {
    lastFrom = 4;
  }
  time_since_last_msg = millis() - time_of_last_msg;
}
void ffMessenger::update(bool verbose, ffGPS myGPS) {
  // Update my location
  if (myGPS.fixquality == B0) {
    outPacket.fixquality = 0;
    if (verbose) Serial.println("Messenger: No GPS Fix");
  }
  if (myGPS.fixquality > B0) {
    //   char lat;                 ///< N/S
    // char lon;                 ///< E/W

    if (myGPS.lat == 'N') {
      outPacket.latitude_fixed = myGPS.latitude_fixed;
    }
    if (myGPS.lat == 'S') {
      outPacket.latitude_fixed = myGPS.latitude_fixed * -1;
    }
    if (myGPS.lon == 'E') {
      outPacket.longitude_fixed = myGPS.longitude_fixed;
    }
    if (myGPS.lon == 'W') {
      outPacket.longitude_fixed = myGPS.longitude_fixed * -1;
    }
    // outPacket.longitude_fixed = myGPS.longitude_fixed;
    outPacket.fixquality = myGPS.fixquality;
    outPacket.satellites = myGPS.satellites;
    outPacket.HDOP = myGPS.HDOP;

    // Copy to message database
    friend_msgs[myAddy].fixquality = outPacket.fixquality;
    friend_msgs[myAddy].latitude_fixed = outPacket.latitude_fixed;
    friend_msgs[myAddy].longitude_fixed = outPacket.longitude_fixed;
    // // debug unit test thing
    //       outPacket.latitude_fixed = 361375000;
    //   outPacket.longitude_fixed = 867851560;
    //   outPacket.fixquality = 1;

    if (verbose) Serial.println("Messenger: GPS Fix, updated outPacket");
  }

  // Update friendDB with distances and headings
  float divisor = 10000000.0;
  // First Update My own data
  if (friend_msgs[myAddy].fixquality != 0) {
    friend_locs[myAddy].latitude = friend_msgs[myAddy].latitude_fixed / divisor;
    friend_locs[myAddy].longitude =
        friend_msgs[myAddy].longitude_fixed / divisor;
    friend_locs[myAddy].distance_meters =
        haversine(friend_locs[myAddy].latitude, friend_locs[myAddy].longitude,
                  friend_locs[myAddy].latitude, friend_locs[myAddy].longitude);
    friend_locs[myAddy].bearing =
        bearing(friend_locs[myAddy].latitude, friend_locs[myAddy].longitude,
                friend_locs[myAddy].latitude, friend_locs[myAddy].longitude);
  }

  // update the whole thing now
  for (int i = 0; i < 10; i++) {
    // skip if bad fix
    if (friend_msgs[i].fixquality == 0) {
      continue;
    }

    friend_locs[i].latitude = friend_msgs[i].latitude_fixed / divisor;
    friend_locs[i].longitude = friend_msgs[i].longitude_fixed / divisor;

    friend_locs[i].distance_meters =
        haversine(friend_locs[myAddy].latitude, friend_locs[myAddy].longitude,
                  friend_locs[i].latitude, friend_locs[i].longitude);
    friend_locs[i].bearing =
        bearing(friend_locs[myAddy].latitude, friend_locs[myAddy].longitude,
                friend_locs[i].latitude, friend_locs[i].longitude);
    if (verbose) {
      Serial.print("Friend ");
      Serial.print(i);
      Serial.print(" distance/bearing ");
      Serial.print(friend_locs[i].distance_meters);
      Serial.print(" / ");
      Serial.print(friend_locs[i].bearing);
      Serial.println(" ");
    }
  }
}
void ffMessenger::send(bool verbose, uint8_t to) {
  if (verbose) {
    Serial.print("Sending to : 0x");
    Serial.print(to, HEX);
    Serial.print(": ");
    for (int i = 0; i < 12; i++) {
      Serial.print(outBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  // Send a reply back to the originator client
  if (verbose) Serial.println("sending message..");
  if (!RHReliableDatagram::sendtoWait((uint8_t*)&outPacket, sizeof(outPacket),
                                      to)) {
    if (verbose && to != 255)
      Serial.println("Messenger Send: Did not recieve ack.");
    if (verbose && to == 255)
      Serial.println("Messenger Send: Message Broadcast, no ack expected.");
  } else {
    if (verbose && to != 255)
      Serial.println("Messenger Send: Delivery Confirmed");
    if (verbose && to == 255)
      Serial.println("Messenger Send: Message Broadcast, no ack expected.");
  }
}
uint32_t ffMessenger::haversine(float lat1, float lon1, float lat2,
                                float lon2) {
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;

  // convert to radians
  lat1 = (lat1)*M_PI / 180.0;
  lat2 = (lat2)*M_PI / 180.0;

  // apply formulae
  double a =
      pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
  double rad = 6371;
  double c = 2 * asin(sqrt(a));
  uint32_t int_meters = round(rad * c * 1000);
  return int_meters;
}
int ffMessenger::bearing(float lat1, float lon1, float lat2, float lon2) {
  float teta1 = radians(lat1);
  float teta2 = radians(lat2);
  float delta1 = radians(lat2 - lat1);
  float delta2 = radians(lon2 - lon1);

  //==================Heading Formula Calculation================//

  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
  float brng = atan2(y, x);
  brng = degrees(brng);  // radians to degrees
  brng = (((int)brng + 360) % 360);
  return brng;
}
// int dumbDistance(uint32_t lat1, uint32_t lon1, uint32_t lat2, uint32_t lon2) {
//   uint32_t dLonSq = (lon2 - lon1) ^ 2;
//   uint32_t dLatSq = (lat2 - lat1) ^ 2;
//   uint32_t dSum = dLonSq + dLatSq;
//   long d = sqrt(dSum);
//   d = (d * 0.08);
//   return d;
// }

#pragma endregion

#pragma region ffIMU
// IMU
ffIMU::ffIMU() : Adafruit_BNO055(55) { system = gyro = accel = mag = 0; }
void ffIMU::startup(bool verbose) {
  if (verbose) Serial.println("IMU Startup...");

  /* Initialise the sensor */
  bool init = Adafruit_BNO055::begin();
  Adafruit_BNO055::setExtCrystalUse(true);
  if (!init && verbose) Serial.println("IMU Init - FAIL");
  if (init && verbose) {
    Serial.println("IMU Init - OK");

    /* Display some basic information on this sensor */
    ffIMU::displaySensorDetails();

    /* Optional: Display current status */
    ffIMU::displaySensorStatus();
  }
  if (verbose) Serial.println("-End IMU Init-");
}
void ffIMU::displaySensorDetails() {
  sensor_t sensor;
  Adafruit_BNO055::getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" xxx");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" xxx");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void ffIMU::displaySensorStatus() {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  Adafruit_BNO055::getSystemStatus(&system_status, &self_test_results,
                                   &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
void ffIMU::displayCalStatus() {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  Adafruit_BNO055::getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system) {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
void ffIMU::update(bool verbose) {
  Adafruit_BNO055::getEvent(&event);
  Adafruit_BNO055::getCalibration(&system, &gyro, &accel, &mag);
  if (verbose) {
    /* Display the floating point data */
    Serial.print("IMU-");
    Serial.print("\tX: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.println(event.orientation.z, 4);
  }
}
#pragma endregion

#pragma region ffEntanglement
ffEntanglement::ffEntanglement(ffGPS myGPS, ffMessenger myMessenger,
                               ffIMU myIMU) {
  // Get my MAC Address
  selfStatus.macAddress = getMacAddress();
  // Look for this in the known addresses and set color and ffAddress
  for (size_t i = 0;
       i < (sizeof(knownMacAddresses) / sizeof(knownMacAddresses[0])); i++) {
    if (knownMacAddresses[i] == selfStatus.macAddress) {
      selfStatus.ffAddress = i;
      selfStatus.ffColor = friendColors[i];
      break;
    }
  }

  // Load in the other known friends
  for (size_t i = 0;
       i < (sizeof(knownMacAddresses) / sizeof(knownMacAddresses[0])); i++) {
    friendStatus[i].macAddress = knownMacAddresses[i];
    friendStatus[i].ffColor = friendColors[i];
  }
}
void ffEntanglement::entangle(ffStatus aFriend) {}
void ffEntanglement::entangle(ffGPS myGPS) {
  // Get Current GPS Data
  selfStatus.fix_quality = myGPS.fixquality;
  if (myGPS.fixquality == B0) {
    selfStatus.fix_quality = 0;
    // if (verbose) Serial.println("Entanglement: No GPS Fix");
  }
  if (myGPS.fixquality > B0) {
    //   char lat;                 ///< N/S
    // char lon;                 ///< E/W

    if (myGPS.lat == 'N') {
      selfStatus.latitude = myGPS.latitude_fixed;
    }
    if (myGPS.lat == 'S') {
      selfStatus.latitude = myGPS.latitude_fixed * -1;
    }
    if (myGPS.lon == 'E') {
      selfStatus.longitude = myGPS.longitude_fixed;
    }
    if (myGPS.lon == 'W') {
      selfStatus.longitude = myGPS.longitude_fixed * -1;
    }
  }
}
void ffEntanglement::entangle(ffMessenger myMessenger) {}
void ffEntanglement::entangle(ffIMU myIMU) {
  // Get Current IMU Data
  selfStatus.orientation_x = myIMU.event.orientation.x;
  selfStatus.orientation_y = myIMU.event.orientation.y;
  selfStatus.orientation_z = myIMU.event.orientation.z;
}
void ffEntanglement::update(ffGPS myGPS, ffMessenger myMessenger, ffIMU myIMU) {
  entangle(myIMU);
  entangle(myGPS);
  entangle(myMessenger);
}
String ffEntanglement::getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1],
          baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  Serial.print("My MAC Address: ");
  Serial.println(String(baseMacChr));
  return String(baseMacChr);
}
void ffEntanglement::printSelfStatus() {
  Serial.println("====Self Status====");
  Serial.print("MAC Address: ");
  Serial.println(selfStatus.macAddress);
  Serial.print("ffAddress: ");
  Serial.println(selfStatus.ffAddress);
  Serial.print("ffColor: ");
  Serial.println(selfStatus.ffColor);
  Serial.print("messageCounter: ");
  Serial.println(selfStatus.messageCounter);
  Serial.print("fix_quality: ");
  Serial.println(selfStatus.fix_quality);
  Serial.print("latitude: ");
  Serial.println(selfStatus.latitude);
  Serial.print("longitude: ");
  Serial.println(selfStatus.longitude);
  Serial.print("orientation_x: ");
  Serial.println(selfStatus.orientation_x);
  Serial.print("orientation_y: ");
  Serial.println(selfStatus.orientation_y);
  Serial.print("orientation_z: ");
  Serial.println(selfStatus.orientation_z);
  Serial.print("battery_V: ");
  Serial.println(selfStatus.battery_V);
  Serial.println("===================");
}
#pragma endregion
