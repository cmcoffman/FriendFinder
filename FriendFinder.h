#ifndef _FRIENDFINDER_
#define _FRIENDFINDER_

#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include "esp_system.h"
#include <SPI.h>

#include "FFConfig.h"

class ffGPS : public Adafruit_GPS {
 private:
 public:
  // Constructor
  ffGPS(HardwareSerial* ser);

  // GPS Functions
  void startup(bool verbose = true);
  void print(bool verbose = true);
  void update(bool verbose = true);

  // GPS Data
  // uint8_t hour, minute, seconds, year, month, day;
  //   uint16_t milliseconds;
  //   // Floating point latitude and longitude value in degrees.
  //   float latitude, longitude;
  //   // Fixed point latitude and longitude value with degrees stored in units
  //   of 1/100000 degrees,
  //   // and minutes stored in units of 1/100000 degrees.  See pull #13 for
  //   more details:
  //   //   https://github.com/adafruit/Adafruit-GPS-Library/pull/13
  //   int32_t latitude_fixed, longitude_fixed;
  //   float latitudeDegrees, longitudeDegrees;
  //   float geoidheight, altitude;
  //   float speed, angle, magvariation, HDOP;
  //   char lat, lon, mag;
  //   boolean fix;
  //   uint8_t fixquality, satellites;
};

// Data Packet
struct dataPacket {
  int32_t latitude_fixed;   // 4 bytes
  int32_t longitude_fixed;  // 4 bytes
  uint8_t fixquality;        // 1 byte
  uint8_t satellites;        // 1 byte
  uint16_t HDOP;             // 2 bytes
};

// Friends Loc and Distance
struct friendDB {
  float latitude;
  float longitude;
  float distance_meters;
  float bearing;
  int age_seconds;
  int quality;
};

class ffRadio : public RH_RF95 {
 private:
 public:
  // Constructor
  ffRadio(uint8_t csPin, uint8_t intPin);
  void startup(bool verbose = true);
  // void print(bool verbose);
  // void update(bool verbose);
};

//#ifdef FFMK2
class ffDisplay : public TFT_eSPI {
 private:
 public:
  // Constructor
  ffDisplay();
  void startup(bool verbose = true);
  // void print(bool verbose);
  // void update(bool verbose);
};
//#endif

class ffMessenger : public RHReliableDatagram {
 private:
 public:
  // Constructor
  ffMessenger(RHGenericDriver& driver, uint8_t thisAddress);

  // Methods
  void startup(bool verbose = true);
  void printPacket(dataPacket packet);
  void check(bool verbose = true);
  void update(bool verbose, ffGPS myGPS);
  void send(bool verbose, uint8_t to);

  uint32_t haversine(float lat1, float lon1, float lat2, float lon2);

  int bearing(float lat1, float lon1, float lat2,
                           float lon2);

  // Ingoing and Outgoing Datapackets
  dataPacket inPacket;
  dataPacket outPacket;

  // Friend Message Array (holds ten friends as is)
  dataPacket friend_msgs[10];

  // Latest message form
  uint8_t lastFrom;

  // Time since last Message
  unsigned long time_of_last_msg;
  unsigned long time_of_last_check;
  unsigned long time_since_last_msg;

  // Summarized Friend Location Data
  friendDB friend_locs[10];

  // Message Recieve Buffer
  uint8_t inBuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t outBuf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(inBuf);

  // My address
  uint8_t myAddy;
};

class ffIMU : public Adafruit_BNO055 {
 private:
 public:
  // Constructor
  ffIMU();

  // Methods
  void startup(bool verbose = true);
  void update(bool verbose = true);
  void displaySensorDetails();
  void displaySensorStatus();
  void displayCalStatus();

  // Event
  sensors_event_t event;

  // Calibration Status
  uint8_t system, gyro, accel, mag;
};

#endif  // Close Library
