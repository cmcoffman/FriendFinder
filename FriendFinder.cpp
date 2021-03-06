#include "FriendFinder.h"

#include <math.h>

// wrapper on Adafruit_GPS constructor
ffGPS::ffGPS(HardwareSerial* ser) : Adafruit_GPS(ser) {}

void ffGPS::startup(bool verbose) {
  if (verbose) Serial.println("GPS Startup...");
#ifdef FFMK2
  GPSSerial.begin(9600, SERIAL_8N1, GPSRX, GPSTX);

  // if(!GPSSerial.begin(9600, SERIAL_8N1, GPSRX, GPSTX)) {
  //   Serial.println("GPS - FAIL");
  // } else {
  //   Serial.println("GPS - OK");
  // }

#else
  Adafruit_GPS::begin(9600);
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
      // Decided not to print a message if there's nothing.
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

// Display Stuff
//#ifdef FFMK2
ffDisplay::ffDisplay() : TFT_eSPI() {}

void ffDisplay::startup(bool verbose) {
  // Setup Display
  if (verbose) Serial.println("Display Startup...");
  ffDisplay::fillScreen(TFT_BLACK);
  TFT_eSPI::init();
  ffDisplay::fillScreen(TFT_PINK);
  delay(100);
  TFT_eSPI::fillScreen(TFT_YELLOW);
  delay(100);
  TFT_eSPI::fillScreen(TFT_LIGHTGREY);
  delay(100);
  TFT_eSPI::fillScreen(TFT_BLACK);
  TFT_eSPI::setRotation(1);  // 1 is USB on right
                             // 2 is USB on top
                             // 3 is USB on left
                             // 4 is USB on bottom
                             // 0 - bottom; 5 - right
  // TFT_eSPI::setCursor(1,1);
}

// void ffDisplay::drawPage(int page) {
//   if (page_change) {
//     fillScreen(TFT_BLACK);
//   }
//   if (page != 1) {
//     digitalWrite(TFT_BL, HIGH);
//   }

// #define NUM_PAGES 3
//   switch (page) {
//     case 1:
//       ffDisplay::screen_off();
//       page_change = false;
//       break;
//     case 2:
//       ffDisplay::screen_splash();
//       page_change = false;
//       break;
//     case 3:
//       ffDisplay::screen_IMU();
//       page_change = false;
//       break;
//     default:
//       ffDisplay::screen_splash();
//       page_change = false;
//       break;
//   }
// }

// void ffDisplay::screen_off() {
//   fillScreen(TFT_BLACK);
//   digitalWrite(TFT_BL, LOW);
// }

// void ffDisplay::screen_splash() {
//   byte font = 1;
//   setTextFont(font);
//   setTextSize(3);
//   // tft.fillScreen(TFT_BLACK);
//   int padding = textWidth("999", font);  // get the width of the text in pixels
//   setTextColor(TFT_GREEN, TFT_BLUE);
//   setTextPadding(padding);
//   // setTextColor(TFT_BLACK, TFT_RED);
//   drawCentreString("FriendFinder", TFT_HEIGHT / 2, TFT_WIDTH / 2, 1);
//   // drawRightString("FriendFinder", 150, 50, 1);
// }

// void ffDisplay::screen_IMU(ffIMU myIMU) {
//   int font = 1;
//   int size = 2;
//   setTextFont(font);
//   setTextSize(size);
//   int fontHeight = fontHeight(font);
//   int padding = textWidth("999", font);  // get the width of the text in pixels
//   setTextColor(TFT_BLACK, TFT_LIGHTGREY);
//   setTextPadding(padding);

//   // Draw the static parts just once
//   if (page_change) {
//     // Background
//     fillScreen(TFT_LIGHTGREY);
//     // Text Labels
//     drawString("   ORI", 0, 3 + fontHeight * 0, 1);
//     drawString("X:", 3, fontHeight * 1, 1);
//     drawString("Y:", 3, fontHeight * 2, 1);
//     drawString("Z:", 3, fontHeight * 3, 1);
//     drawString("Lat:", 3, fontHeight * 4, 1);
//     drawString("Lon:", 3, fontHeight * 5, 1);
//   }

//   int x = random(0, 360);
//   int y = random(0, 360);
//   int z = random(0, 360);

//   char buf[5];
//   dtostrf(myIMU.event.orientation.x, 4, 0, buf);
//   setTextColor(TFT_RED, TFT_LIGHTGREY);
//   drawRightString(buf, textWidth(buf) + textWidth("X:"), 3 + fontHeight * 1, 1);

//   // char buf[3];
//   dtostrf(myIMU.event.orientation.y, 4, 0, buf);
//   setTextColor(TFT_DARKGREEN, TFT_LIGHTGREY);
//   drawRightString(buf, textWidth(buf) + textWidth("X:"), 3 + fontHeight * 2, 1);

//   // char buf[3];
//   dtostrf(myIMU.event.orientation.z, 4, 0, buf);
//   setTextColor(TFT_BLUE, TFT_LIGHTGREY);
//   drawRightString(buf, textWidth(buf) + textWidth("X:"), 3 + fontHeight * 3, 1);
// }
// //#endif

// Radio Stuff
// wrapper on RH_RF95 constructor
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

// Messaging Stuff
// wrapper on RHReliableDatagram constructor
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

// This should build the outPacket (with my status)
// and updates the friendDB with distances and headings
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
  // just broadcast
  // to = 255;
  // Serial.println("copying data to outbuffer..");
  // memcpy(&inPacket, inBuf, sizeof(inPacket));
  // memcpy(&outBuf, &outPacket, sizeof(outPacket));
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

// float ffMessenger::calcDistance(uint32_t my_lat, uint32_t my_long,
//                                 uint32_t their_lat, uint32_t their_long) {
//   // float TinyGPS::distance_between (float lat1, float long1, float lat2,
//   float
//   // long2)
//   // returns value in meters
//   my_lat /= 10000000;
//   my_long /= 10000000;
//   their_lat /= 10000000;
//   their_long /= 10000000;
//   float distance =
//       TinyGPS::distance_between(my_lat, my_long, their_lat, their_long);
//   return distance;
// }

uint32_t ffMessenger::haversine(float lat1, float lon1, float lat2,
                                float lon2) {
  // cribbed from
  // https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
  // convert back to floating point
  // lat1 /= 10000000;
  // lon1 /= 10000000;
  // lat2 /= 10000000;
  // lon2 /= 10000000;
  // distance between latitudes
  // and longitudes
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

// // Haversine Distance Calculator with Fixed width coords
// // from: http://forum.arduino.cc/index.php?topic=45760.msg332014#msg332014
// int ffMessenger::HaverSineFixed(uint32_t lat1, uint32_t lon1, uint32_t lat2,
//                                 uint32_t lon2) {
//   float flat1 = lat1 / 10000000;
//   float flat2 = lat2 / 10000000;
//   float flon1 = lon1 / 10000000;
//   float flon2 = lon2 / 10000000;

//   float ToRad = PI / 180.0;
//   float R = 63710;  // radius earth in m

//   float dLat = (lat2 - lat1) * ToRad;
//   float dLon = (lon2 - lon1) * ToRad;

//   float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1 * ToRad) *
//                                                 cos(lat2 * ToRad) *
//                                                 sin(dLon / 2) * sin(dLon /
//                                                 2);

//   float c = 2 * atan2(sqrt(a), sqrt(1 - a));

//   float d = ((R * c) * 8) / 1000;  // convert to meters
//   int distance = d;
//   return distance;
// }

int ffMessenger::bearing(float lat1, float lon1, float lat2, float lon2) {
  // Gotten from:
  // https://gis.stackexchange.com/questions/252672/calculate-bearing-between-two-decimal-gps-coordinates-arduino-c?newreg=eb676d9dca8f4cc8ad10c14a3b00d423

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

  // Serial.print("Heading GPS: ");
  // Serial.println(brng);

  return brng;
}

int dumbDistance(uint32_t lat1, uint32_t lon1, uint32_t lat2, uint32_t lon2) {
  uint32_t dLonSq = (lon2 - lon1) ^ 2;
  uint32_t dLatSq = (lat2 - lat1) ^ 2;
  uint32_t dSum = dLonSq + dLatSq;
  long d = sqrt(dSum);
  d = (d * 0.08);
  return d;
}

// Compass Stuff
// wrapper on Adafruit_Sensor constructor
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

ffEntanglement::ffEntanglement(ffGPS myGPS, ffMessenger myMessenger,
                               ffIMU myIMU) {}

void ffEntanglement::update(ffGPS myGPS,
                            ffMessenger myMessenger,
                            ffIMU myIMU) {

  self.fix_quality = myGPS.fixquality;
  
  if (myGPS.fixquality == B0) {
    self.fix_quality = 0;
   // if (verbose) Serial.println("Entanglement: No GPS Fix");
  }
  if (myGPS.fixquality > B0) {
    //   char lat;                 ///< N/S
    // char lon;                 ///< E/W

    if (myGPS.lat == 'N') {
      self.latitude = myGPS.latitude_fixed;
    }
    if (myGPS.lat == 'S') {
      self.latitude = myGPS.latitude_fixed * -1;
    }
    if (myGPS.lon == 'E') {
      self.longitude = myGPS.longitude_fixed;
    }
    if (myGPS.lon == 'W') {
      self.longitude = myGPS.longitude_fixed * -1;
    }
  }
  self.orientation_x = myIMU.event.orientation.x;
  self.orientation_y = myIMU.event.orientation.y;
  self.orientation_z = myIMU.event.orientation.z;
}

// // Buttons to work on
// #ifdef FFMK2
// ffButton::ffButton() : Button2() {}

// void ffIMU::startup(bool verbose) {
//   if (verbose) Serial.println("IMU Startup...");

//   /* Initialise the sensor */
//   bool init = Adafruit_BNO055::begin();
//   Adafruit_BNO055::setExtCrystalUse(true);
//   if (!init && verbose) Serial.println("IMU Init - FAIL");
//   if (init && verbose) {
//     Serial.println("IMU Init - OK");

//     /* Display some basic information on this sensor */
//     ffIMU::displaySensorDetails();

//     /* Optional: Display current status */
//     ffIMU::displaySensorStatus();
//   }
//   if (verbose) Serial.println("-End IMU Init-");
// }
