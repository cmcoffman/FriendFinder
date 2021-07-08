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
  
  uint8_t primary_IMU;  // 1 = BNO055 ; 2 = BNO085
  float orientation_x;  // Consensus data
  float orientation_y;  // Consensus data
  float orientation_z;  // Consensus data
  
  
  float IMU055_orientation_x;  // BNO055 data
  float IMU055_orientation_y;  // BNO055 data
  float IMU055_orientation_z;  // BNO055 data
  
  float IMU085_orientation_x;  // BNO085 data
  float IMU085_orientation_y;  // BNO085 data
  float IMU085_orientation_z;  // BNO085 data

  float IMU085_rotationVector_real;
  float IMU085_rotationVector_i;
  float IMU085_rotationVector_j;
  float IMU085_rotationVector_k;
  
  float IMU085_rotationVector_yaw;
  float IMU085_rotationVector_pitch;
  float IMU085_rotationVector_roll;


  float battery_V;
};

// Serial.print(sensorValue.un.rotationVector.real);
// Serial.print(sensorValue.un.rotationVector.i);
// Serial.print(sensorValue.un.rotationVector.j);
// Serial.print(sensorValue.un.rotationVector.k);

// sensorValue.un.rotationVector.real
// sensorValue.un.rotationVector.i
// sensorValue.un.rotationVector.j
// sensorValue.un.rotationVector.k

// float rotationVector_real
// float rotationVector_i
// float rotationVector_j
// float rotationVector_k