
// Neopixel Ring Stuff-----
// Fast LED
#define DATA_PIN 5
//#define CLK_PIN   4
#define LED_TYPE WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS 24

#define BRIGHTNESS 96
#define FRAMES_PER_SECOND 120

#define NUMPIXELS 24
// Strip is connected to Arduino Pin 5
#define NEOPIXEL_RING_PIN 5

// Neopixel Ring orientation relative to IMU
#define TOPPIXEL 15
// Is the IMU upside down relative to the ring?
#define IMU_INVERT 1

// GPS Stuff-----
#define GPSSerial Serial1


// Radio Stuff-----
// for Prototype Board
#define RFM95_CS 6
#define RFM95_RST 11
#define RFM95_INT 10

// #define MY_ADDRESS PROTBOARD_ADDRESS
// #define FRIEND_ADDRESS ESP32_ADDRESS

// Change to 434.0 or other frequency, must match RX's freq!


