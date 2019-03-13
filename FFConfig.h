#ifndef _FRIENDFINDERCONFIG_
#define _FRIENDFINDERCONFIG_


#ifdef ARDUINO_FEATHER_ESP32
// Neopixel Ring Stuff-----
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 12


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


#define MY_ADDRESS 1
//#define BEACON_ADDRESS 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#endif

#ifdef ARDUINO_SAMD_FEATHER_M0_EXPRESS
// Neopixel Ring Stuff-----
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 5


// GPS Stuff-----
#define GPSSerial Serial1


// Radio Stuff-----
// for Prototype Board
#define RFM95_CS 6
#define RFM95_RST 11
#define RFM95_INT 10

#define MY_ADDRESS 2
//#define BEACON_ADDRESS 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0


#endif

#ifdef ARDUINO_AVR_FEATHER32U4
// Neopixel Ring Stuff-----
#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 5


// GPS Stuff-----
#define GPSSerial Serial1


// Radio Stuff-----
// // for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7


#define MY_ADDRESS 3
//#define BEACON_ADDRESS 2

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

#endif  // Close Library
