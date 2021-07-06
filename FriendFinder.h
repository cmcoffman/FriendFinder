// Status
struct ffStatus {
  String macAddress;
  uint8_t ffAddress;           // This is the address for the radio/datagram too
  uint16_t ffColor;            // For displays/neopixels
  uint8_t messageCounter = 0;  // total messages recieved from friend
  uint8_t fixquality;
  uint8_t fixquality_3d;
  float latitude;
  float longitude;
  int32_t latitude_fixed;  ///< Fixed point latitude in decimal degrees.
                           ///< Divide by 10000000.0 to get a double.
  int32_t longitude_fixed; ///< Fixed point longitude in decimal degrees
                           ///< Divide by 10000000.0 to get a double.
  float orientation_x;
  float orientation_y;
  float orientation_z;
  float battery_V;
};