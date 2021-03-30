#ifndef _FFMK2_
#define _FFMK2_
// FFMK2-Display.h
#define FFMK2
#include <TFT_eSPI.h>


// Display Stuff-----
//TFT_eSPI tft = TFT_eSPI();  // x = 135, y = 240

// GPS Stuff-----
#define GPSSerial Serial2
#define GPSRX 37
#define GPSTX 38

// Radio Stuff-----
#define MY_ADDRESS 5
#define THEIR_ADDRESS 1
#define RFM95_CS 33
#define RFM95_RST 2
#define RFM95_INT 32

#define HSPI_SCK 25
#define HSPI_MISO 27
#define HSPI_MOSI 26

// Button Stuff ----
#define BUTTON_A_PIN 0
#define BUTTON_B_PIN 35


#endif // close