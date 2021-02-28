//#include <hardware_setups/FFMK1.h>  // Setup file for mk1 prototype (protoboard)
#define FFMK1
// Neopixel Ring Stuff-----
//#define CLK_PIN   4
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS 24

#define BRIGHTNESS 96
#define FRAMES_PER_SECOND 120

#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
//Breadboard Prototype
//#define NEOPIXEL_RING_PIN 20

// FF_PCB
#define NEOPIXEL_RING_PIN A0

// Neopixel Ring orientation relative to IMU
// Is the IMU upside down relative to the ring?
#define IMU_INVERT 0


#define TOPPIXEL 10

// FastLED
#define DATA_PIN A0

// GPS Stuff-----
#define GPSSerial Serial2


// Radio Stuff-----
// for ESP32 Board
#define RFM95_CS 33
#define RFM95_RST 27
#define RFM95_INT 21

