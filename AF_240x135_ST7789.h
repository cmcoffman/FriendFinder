

#pragma message( "Using display: AF_240x135_ST7789.h (local folder)" )
#define ST7789_DRIVER
#define TFT_SDA_READ   // Display has a bidirectionsl SDA pin

#define TFT_WIDTH  135
#define TFT_HEIGHT 240

#define CGRAM_OFFSET      // Library will add offsets required

//#define USE_HSPI_PORT
#define TFT_MISO            19  //"MISO" on esp32 feather
#define TFT_MOSI            18  //"MOSI" on esp32 feather  
#define TFT_SCLK            5   //"SCK" on esp32 feather 
#define TFT_CS              27  //"27" on esp32 feather
#define TFT_DC              33  //"33" on esp32 feather
#define TFT_RST             32  //"32" on esp32 feather

#define TFT_BL              25  // (A1) Display backlight control pin

#define TFT_BACKLIGHT_ON HIGH  // HIGH or LOW are options

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

#define SMOOTH_FONT

#define SPI_FREQUENCY  27000000
//  #define SPI_FREQUENCY  40000000   // Maximum for ILI9341


#define SPI_READ_FREQUENCY  6000000 // 6 MHz is the maximum SPI read speed for the ST7789V
