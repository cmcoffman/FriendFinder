// Types
// typedef enum
// {

//   WIFI_CONNECTION_TIMEOUT = 1,
//   WIFI_REASON_AUTH_EXPIRE = 2,
//   WIFI_REASON_AUTH_LEAVE = 3,
//   WIFI_REASON_ASSOC_EXPIRE = 4,
//   WIFI_REASON_ASSOC_TOOMANY = 5,
//   WIFI_REASON_NOT_AUTHED = 6,

// } status_codes;

// Status Object
struct ffStatus
{
  String macAddress;
  uint8_t ffAddress;          // This is the address for the radio/datagram too
  uint16_t ffColor;           // For displays/neopixels
  uint8_t messageCounter = 0; // total messages recieved from friend

  // GPS
  bool GPS_connected = false;
  bool GPS_naive = true;   // Has a fix ever been made? true = no
  bool GPS_fix = false;    ///
  uint8_t fixquality;      ///< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
  uint8_t fixquality_3d;   ///< 3D fix quality (1, 3, 3 = Nofix, 2D fix, 3D fix)
  uint8_t satellites;      ///< Number of satellites in use
  int32_t latitude_fixed;  ///< Fixed point latitude in decimal degrees.
  int32_t longitude_fixed; ///< Fixed point longitude in decimal degrees
  float latitude;          // Above divided by 10000000.0 to get a double
  float longitude;         // Above divided by 10000000.0 to get a double

  // IMU
  uint8_t primary_IMU; // 1 = BNO055 ; 2 = BNO085
  float orientation_x; // Consensus data
  float orientation_y; // Consensus data
  float orientation_z; // Consensus data

  bool IMU085_connected = false;
  bool IMU085_reports_set = false;
  float IMU085_orientation_x; // BNO085 data
  float IMU085_orientation_y; // BNO085 data
  float IMU085_orientation_z; // BNO085 data

  float IMU085_rotationVector_real;
  float IMU085_rotationVector_i;
  float IMU085_rotationVector_j;
  float IMU085_rotationVector_k;

  float IMU085_rotationVector_yaw;
  float IMU085_rotationVector_pitch;
  float IMU085_rotationVector_roll;

  // Radio
    bool LORA_connected = false;

  // Knob
  int knob_V;
  int knob_mode;

  // Display
  bool display_enable = false;

  // LED
  bool LED_enable = true;

  // OTA/ WiFi
  bool OTA_enable = true;
  bool OTA_timeout = false;
  TickType_t xTicks_When_Wifi_Disconnected = 0;

  // Battery
  float battery_V;
};

/* Display Colors
Tetradic color pallete based on TFT_ORANGE (255, 180, 0)
Color           RGB565    RGB             HEX    
FFPAL_ORANGE    0xFDA0    255, 180,   0   #ffb400
FFPAL_GREEN     0x07E6      0, 255,  53   #00ff35
FFPAL_BLUE      0x025F      0,  75, 255   #004bff
FFPAL_PURPLE    0xF819    255,   0, 202   #ff00ca

*/
// FF Colors
#define FFPAL_ORANGE 0xFDA0
#define FFPAL_GREEN 0x07E6
#define FFPAL_BLUE 0x025F
#define FFPAL_PURPLE 0xF819

// #pragma region Display // Display Config

// #define USER_SETUP_LOADED

// #define ST7789_DRIVER
// #define TFT_SDA_READ   // Display has a bidirectionsl SDA pin

// #define TFT_WIDTH  135
// #define TFT_HEIGHT 240

// #define CGRAM_OFFSET      // Library will add offsets required

// #define TFT_MISO 19

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

// #pragma endregion
