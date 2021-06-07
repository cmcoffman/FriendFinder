#ifndef _FFMK3_
#define _FFMK3_
// FFMK2-Display.h
#define FFMK3
// //#include <TFT_eSPI.h>

// // Display Stuff-----
// #define USER_SETUP_LOADED 

// #define ST7789_DRIVER
// #define TFT_SDA_READ   // Display has a bidirectionsl SDA pin

// #define TFT_WIDTH  135
// #define TFT_HEIGHT 240

// #define CGRAM_OFFSET      // Library will add offsets required

// //#define TFT_MISO -1

// #define TFT_MOSI            18
// #define TFT_SCLK            5
// #define TFT_CS              27
// #define TFT_DC              33
// #define TFT_RST             32

// #define TFT_BL          25  // (A1) Display backlight control pin

// #define TFT_BACKLIGHT_ON HIGH  // HIGH or LOW are options

// #define LOAD_GLCD
// #define LOAD_FONT2
// #define LOAD_FONT4
// #define LOAD_FONT6
// #define LOAD_FONT7
// #define LOAD_FONT8
// #define LOAD_GFXFF

// #define SMOOTH_FONT

// //#define SPI_FREQUENCY  27000000
//   #define SPI_FREQUENCY  40000000   // Maximum for ILI9341


// #define SPI_READ_FREQUENCY  6000000 // 6 MHz is the maximum SPI read speed for the ST7789V






// GPS Stuff-----
#define GPSSerial Serial2
#define GPSRX 16
#define GPSTX 17

// Radio Stuff-----
#define MY_ADDRESS 5
#define THEIR_ADDRESS 1
#define RFM95_CS 33
#define RFM95_RST 2
#define RFM95_INT 32

// #define HSPI_SCK 5
// #define HSPI_MISO 19
// #define HSPI_MOSI 18

// Button Stuff ----
// #define BUTTON_A_PIN 0
// #define BUTTON_B_PIN 35


#endif // close