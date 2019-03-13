// ffNeoRing.cpp
// a wrapper/dervied class of Adafruit_NeoPixel to add some functionality
// most everything passes on to the parent class

#include "FriendFinder.h"
#include <Adafruit_NeoPixel.h>

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
void ffNeoRing::colorDotWipe(uint32_t c, uint8_t wait) {
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
void ffNeoRing::colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < numPixels(); i++) {
    setPixelColor(i, c);
    show();
    delay(wait);
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
  }
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

  if (verbose) Serial.println("Set Radio power...");
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
  if (RHReliableDatagram::available()) {
    // Wait for a message addressed to us from the client
    // uint8_t len = sizeof(buf);
    // What address is this from?
    uint8_t from;
    if (RHReliableDatagram::recvfromAck(inBuf, &len)) {
      // copy message to inpacket
      memcpy(&inPacket, inBuf, sizeof(inPacket));
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


    if (verbose && to != 255) Serial.println("Messenger Send: Did not recieve ack.");
    if (verbose && to == 255) Serial.println("Messenger Send: Message Broadcast, no ack expected.");
  } else {
    if (verbose && to != 255) Serial.println("Messenger Send: Delivery Confirmed");
    if (verbose && to == 255) Serial.println("Messenger Send: Message Broadcast, no ack expected.");
  }

}
