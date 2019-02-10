// IMU Stuff
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Sample Rate
#define BNO055_SAMPLERATE_DELAY_MS (100)

float heading;

sensors_event_t event;

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  Serial.println("--------------------");
  Serial.println("IMU Sensor Status:");
  Serial.println("================");
  /* Display the results in the Serial Monitor */
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("--------------------");
  //delay(500);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) {
  Serial.println("--------------------");
  Serial.println("IMU Sensor Offsets:");
  Serial.println("================");
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
  Serial.println("");
  Serial.println("--------------------");
}

void initIMU() {
  int now = millis();
  Serial.print("IMU");
  while (!bno.begin()) {
    // Sensor not working
    Serial.print(".");
    colorDotWipe(RED, 10);
  }
  if (bno.begin()) {
    Serial.println("...[OK]");
    colorDotWipe(GREEN, 10);
  }
}



int getMagStatus(void) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /* The data should be ignored until the system calibration is > 0 */
  if (!system)
  {
    return (0);
  }
  /* Display the individual values */
  return ((int) mag);
}

bool checkMag() {
  if (getMagStatus() != 3) {
    return false;
  } else {
    return true;
  }
}



void waitForMag() {
  // wait for magnetometer calibration (move it around)
  Serial.print("Magnetometer Calibration...");
  while (getMagStatus() != 3) {
    colorDotWipe(YELLOW, 40);
    Serial.print(".");
  }
  Serial.println("[OK]");
  // Display Sensor offsets
  displaySensorStatus();
  // show calibration results
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);
}

// Compass Stuff ##########################


void checkIMU() {
  bno.getEvent(&event);
}



int findNorth() {
  return event.orientation.x;
}
