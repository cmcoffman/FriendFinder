// ffNeoRing.cpp
// a wrapper/dervied class of Adafruit_NeoPixel to add some functionality
// most everything passes on to the parent class

#include "FriendFinder.h"
#include <Adafruit_NeoPixel.h>
#include <TinyGPS.h>
#include <math.h>

const uint32_t ffNeoRing::Red = Adafruit_NeoPixel::Color(255, 0, 0);
const uint32_t ffNeoRing::Green = Adafruit_NeoPixel::Color(0, 255, 0);
const uint32_t ffNeoRing::Blue = Adafruit_NeoPixel::Color(0, 0, 255);
const uint32_t ffNeoRing::Yellow = Adafruit_NeoPixel::Color(255, 255, 0);
const uint32_t ffNeoRing::White = Adafruit_NeoPixel::Color(255, 255, 255);
const uint32_t ffNeoRing::Grey = Adafruit_NeoPixel::Color(64, 64, 64);
const uint32_t ffNeoRing::Off = Adafruit_NeoPixel::Color(0, 0, 0);
const uint32_t ffNeoRing::YellowGreen = Adafruit_NeoPixel::Color(128, 255, 0);
const uint32_t ffNeoRing::Purple = Adafruit_NeoPixel::Color(255, 0, 255);

// wrapper on Adafruit_NeoPixel constructor
ffNeoRing::ffNeoRing(uint16_t n, uint8_t p, uint8_t t)
    : Adafruit_NeoPixel(n, p, t) {}

// Color Dot Runs around ring
void ffNeoRing::colorDotWipe(uint32_t c, uint16_t wait) {
  for (uint16_t i = 0; i < numPixels(); i++) {
    setPixelColor(i, c);
    setPixelColor(i - 1, Adafruit_NeoPixel::Color(0, 0, 0));
    show();
    delay(wait);
  }
  setPixelColor(numPixels() - 1, Adafruit_NeoPixel::Color(0, 0, 0));
  show();
}

// Fill Whole Ring with color
void ffNeoRing::colorWipe(uint32_t c, uint16_t wait) {
  for (uint16_t i = 0; i < numPixels(); i++) {
    setPixelColor(i, c);
    show();
    delayMicroseconds(wait);
  }
}

void ffNeoRing::colorDot(int pixel, uint32_t color) {
  for (uint16_t i = 0; i < numPixels(); i++) {
    if (i == pixel) {
      setPixelColor(i, color);
    } else {
      setPixelColor(i, Off);
    }
    show();
    delay(2);
  }
}

void ffNeoRing::flash() {
  for (int j = 255; j > 0; j--) {
    for (uint16_t i = 0; i < numPixels(); i++) {
      setPixelColor(i, Color(j / 1.1, j / 1.1, j / 1.1));
    }
    delayMicroseconds(50);
    show();
  }
  delay(1);
  ffNeoRing::clearStrip();
  //ffNeoRing::show();
  show();
}

// overload the base class show to check if stripChanged
void ffNeoRing::show(void) { Adafruit_NeoPixel::show(); }

void ffNeoRing::clearStrip() {
  for (int i = 0; i < numPixels(); i++) setPixelColor(i, 0);
}

void ffNeoRing::fillStrip(uint32_t c) {
  for (int i = 0; i < numPixels(); i++) setPixelColor(i, c);
}

uint32_t ffNeoRing::randomWheelColor(void) {
  return colorWheel(random(0, 255));
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t ffNeoRing::colorWheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return Adafruit_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return Adafruit_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return Adafruit_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

int ffNeoRing::orientRing(int heading) {
  // Get degrees per pixel
  int degPerPixel = 360 / numPixels();
  // Calculate pixel for heading
  int pixelOut = -((heading / degPerPixel) - TOPPIXEL);
  pixelOut = (pixelOut + 24) % 24;
  return (pixelOut);
}

// wrapper on Adafruit_GPS constructor
ffGPS::ffGPS(HardwareSerial* ser) : Adafruit_GPS(ser) {}

void ffGPS::startup(bool verbose) {
  Adafruit_GPS::begin(9600);
  // turn on only GPRMC sentence
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // turn on GPRMC and GGA
  // Adafruit_GPS::sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // GGA only
  Adafruit_GPS::sendCommand(
      "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
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

// Radio Stuff
// wrapper on RH_RF95 constructor
ffRadio::ffRadio(uint8_t csPin, uint8_t intPin) : RH_RF95(csPin, intPin) {}

void ffRadio::startup(bool verbose) {
  if (verbose) Serial.println("Radio Startup...");

  // Manual Reset
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

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

// Messaging Stuff
// wrapper on RHReliableDatagram constructor
ffMessenger::ffMessenger(RHGenericDriver& driver, uint8_t thisAddress)
    : RHReliableDatagram(driver, thisAddress) {}

void ffMessenger::startup(bool verbose) {
  if (verbose) Serial.println("Messenger Startup...");
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
      // memcpy(&friendDB[from], inBuf, sizeof(inPacket));
      // reset time since last message
      time_of_last_msg = millis();
      if (verbose) {
        Serial.print("got request from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        for (int i = 0; i < (RH_RF95_MAX_MESSAGE_LEN - 1); i++) {
          Serial.print(inBuf[i], HEX);
          // Serial.print(" ");
        }
        Serial.println();
      }

      // // Send a reply back to the originator client
      // if (!manager.sendtoWait(data, sizeof(data), from))
      //   Serial.println("sendtoWait failed");
    }
  }
  time_since_last_msg = millis() - time_of_last_msg;
}

// This should build the outPacket (with my status)
// and eventually also process incoming messages variously
// such as adding timestamps so we know how old the messages are.
void ffMessenger::update(bool verbose, ffGPS myGPS) {
  if (myGPS.fixquality == B0) {
    outPacket.fixquality = 0;
    if (verbose) Serial.println("Messenger: No GPS Fix");
  }
  if (myGPS.fixquality > B0) {
    outPacket.latitude_fixed = myGPS.latitude_fixed;
    outPacket.longitude_fixed = myGPS.longitude_fixed;
    outPacket.fixquality = myGPS.fixquality;
    outPacket.satellites = myGPS.satellites;
    outPacket.HDOP = myGPS.HDOP;
    if (verbose) Serial.println("Messenger: GPS Fix, updated outPacket");
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
  Serial.println("sending message..");
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

float ffMessenger::calcDistance(uint32_t my_lat, uint32_t my_long,
                                uint32_t their_lat, uint32_t their_long) {
  // float TinyGPS::distance_between (float lat1, float long1, float lat2, float
  // long2)
  // returns value in meters
  my_lat /= 10000000;
  my_long /= 10000000;
  their_lat /= 10000000;
  their_long /= 10000000;
  float distance =
      TinyGPS::distance_between(my_lat, my_long, their_lat, their_long);
  return distance;
}

uint16_t ffMessenger::haversine(double lat1, double lon1, double lat2,
                                double lon2) {
  // cribbed from
  // https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
  // convert back to floating point
  lat1 /= 10000000;
  lon1 /= 10000000;
  lat2 /= 10000000;
  lon2 /= 10000000;
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
  uint16_t int_meters = round(rad * c * 1000);
  return int_meters;
}

// Compass Stuff
// wrapper on Adafruit_Sensor constructor
ffIMU::ffIMU() : Adafruit_BNO055(55) {
  system = gyro = accel = mag = 0;
}

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

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
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

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
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

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
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
