#ifndef _FRIENDFINDERCONFIG_
#define _FRIENDFINDERCONFIG_

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

#define ESP32_ADDRESS 1
#define PROTBOARD_ADDRESS 2
#define BEACON_ADDRESS 3

#ifdef ARDUINO_FEATHER_ESP32
// Neopixel Ring Stuff-----
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 12

// Neopixel Ring orientation relative to IMU
#define TOPPIXEL 15


// GPS Stuff-----
#define GPSSerial Serial2


// Radio Stuff-----
// for ESP32 Board
#define RFM95_CS 33
#define RFM95_RST 22
#define RFM95_INT 15

// // for Feather32u4 RFM9x
// #define RFM95_CS 8
// #define RFM95_RST 4
// #define RFM95_INT 7


#define MY_ADDRESS ESP32_ADDRESS
#define FRIEND_ADDRESS PROTBOARD_ADDRESS
//#define BEACON_ADDRESS 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#endif

#ifdef ARDUINO_SAMD_FEATHER_M0_EXPRESS
// Neopixel Ring Stuff-----
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 5

// Neopixel Ring orientation relative to IMU
#define TOPPIXEL 15

// GPS Stuff-----
#define GPSSerial Serial1


// Radio Stuff-----
// for Prototype Board
#define RFM95_CS 6
#define RFM95_RST 11
#define RFM95_INT 10

#define MY_ADDRESS PROTBOARD_ADDRESS
#define FRIEND_ADDRESS ESP32_ADDRESS

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0


#endif

#ifdef ARDUINO_AVR_FEATHER32U4
// Neopixel Ring Stuff-----
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 5

// Neopixel Ring orientation relative to IMU
#define TOPPIXEL 15


// GPS Stuff-----
#define GPSSerial Serial1


// Radio Stuff-----
// // for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7


#define MY_ADDRESS BEACON_ADDRESS
#define FRIEND_ADDRESS 1

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#endif

#endif  // Close Library
