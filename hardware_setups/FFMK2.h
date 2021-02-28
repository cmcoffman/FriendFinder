#ifndef _FFMK2_
#define _FFMK2_
// FFMK2-Display.h
#define FFMK2
#include <TFT_eSPI.h>
#include <Button2.h>

// Display Stuff-----
//TFT_eSPI tft = TFT_eSPI();  // x = 135, y = 240

// GPS Stuff-----
#define GPSSerial Serial2

// Radio Stuff-----
#define MY_ADDRESS 2
#define THEIR_ADDRESS 1
#define RFM95_CS 33
#define RFM95_RST 2
#define RFM95_INT 32

#define HSPI_SCK 25
#define HSPI_MISO 27
#define HSPI_MOSI 26



#endif // close