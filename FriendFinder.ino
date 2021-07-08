#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BNO08x.h>
#include "FriendFinder.h"


// FreeRTOS
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// FF Data
ffStatus self_status;
static SemaphoreHandle_t status_mutex;   // Locks GPS object



// Serial Terminal Output
static SemaphoreHandle_t Serial_mutex;   // Locks Serial object

#pragma region OTA Update
// OTA Update
const char *ssid = "nash_equilibrium";
const char *password = "5squidsw/ink";

void handleOTA(void *parameter) {
  // 'Setup'
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  ArduinoOTA.setHostname("FFProto");

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else  // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
    // using SPIFFS.end()
    Serial.println("Start updating " + type);
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // 'Loop'
  while (1) {
    ArduinoOTA.handle();
  };
}
#pragma endregion

#pragma region GPS
// GPS
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false // Echo GPS to Serial terminal?
bool GPS_connected = false;
static SemaphoreHandle_t GPS_mutex;   // Locks GPS object
static SemaphoreHandle_t GPS_new;     //Signals NEW GPS data

void updateGPS(void *parameter) {
  while (1) {
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    // if (GPSECHO)
    //   if (c)
    //   xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    //     Serial.print(c);
    //   xSemaphoreGive(Serial_mutex);
    // // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      xSemaphoreTake(GPS_mutex, portMAX_DELAY);
      if (GPS.parse(GPS.lastNMEA())) { // sets newNMEAreceived() = false
        xSemaphoreTake(status_mutex, portMAX_DELAY);
        self_status.fixquality = GPS.fixquality;
        self_status.fixquality_3d = GPS.fixquality_3d;
        self_status.latitude_fixed = GPS.latitude_fixed;
        self_status.longitude_fixed = GPS.longitude_fixed;
        self_status.latitude = GPS.latitude_fixed / 10000000.0;
        self_status.longitude = GPS.longitude_fixed / 10000000.0;
        xSemaphoreGive(status_mutex);
        xSemaphoreGive(GPS_new);
      }
    }
    xSemaphoreGive(GPS_mutex);
  }
}

void printGPS(void *parameter) {
  while (1) {
    xSemaphoreTake(GPS_new, portMAX_DELAY);
    xSemaphoreTake(GPS_mutex, portMAX_DELAY);

    // Time in seconds keeps increasing after we get the NMEA sentence.
    // This estimate will lag real time due to transmission and parsing delays,
    // but the lag should be small and should also be consistent.
    float s = GPS.seconds + GPS.milliseconds / 1000. + GPS.secondsSinceTime();
    int m = GPS.minute;
    int h = GPS.hour;
    int d = GPS.day;
    // Adjust time and day forward to account for elapsed time.
    // This will break at month boundaries!!! Humans will have to cope with
    // April 31,32 etc.
    while (s > 60) {
      s -= 60;
      m++;
    }
    while (m > 60) {
      m -= 60;
      h++;
    }
    while (h > 24) {
      h -= 24;
      d++;
    }
    xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    // ISO Standard Date Format, with leading zeros https://xkcd.com/1179/
    Serial.print("\nDate: ");
    Serial.print(GPS.year + 2000, DEC);
    Serial.print("-");
    if (GPS.month < 10)
      Serial.print("0");
    Serial.print(GPS.month, DEC);
    Serial.print("-");
    if (d < 10)
      Serial.print("0");
    Serial.print(d, DEC);
    Serial.print("   Time: ");
    if (h < 10)
      Serial.print("0");
    Serial.print(h, DEC);
    Serial.print(':');
    if (m < 10)
      Serial.print("0");
    Serial.print(m, DEC);
    Serial.print(':');
    if (s < 10)
      Serial.print("0");
    Serial.println(s, 3);
    Serial.print("Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(" quality: ");
    Serial.println((int)GPS.fixquality);
    Serial.print("Time [s] since last fix: ");
    Serial.println(GPS.secondsSinceFix(), 3);
    Serial.print("    since last GPS time: ");
    Serial.println(GPS.secondsSinceTime(), 3);
    Serial.print("    since last GPS date: ");
    Serial.println(GPS.secondsSinceDate(), 3);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4);
      Serial.println(GPS.lon);
      Serial.print("Speed (knots): ");
      Serial.println(GPS.speed);
      Serial.print("Angle: ");
      Serial.println(GPS.angle);
      Serial.print("Altitude: ");
      Serial.println(GPS.altitude);
      Serial.print("Satellites: ");
      Serial.println((int)GPS.satellites);
    }
    xSemaphoreGive(GPS_mutex);
    xSemaphoreGive(Serial_mutex);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
#pragma endregion
#pragma region LED
// LED
static const int led_pin = LED_BUILTIN;
void toggleLED(void *parameter) {
  while (1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(1750 / portTICK_PERIOD_MS);
  }
}
#pragma endregion

#pragma region IMU055
// IMU
Adafruit_BNO055 IMU055 = Adafruit_BNO055(55, 0x28); // id, address
bool IMU055_connected = false;
static SemaphoreHandle_t IMU055_mutex;   // Locks IMU object
static SemaphoreHandle_t IMU055_new;     //Signals NEW IMU data

void handleIMU055(void *parameter) {
  sensors_event_t orientationData , linearAccelData;
  double xPos = 0, yPos = 0, headingVel = 0;
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
  uint16_t PRINT_DELAY_MS = 500; // how often to print the data
  //velocity = accel*dt (dt in seconds)
  //position = 0.5*accel*dt^2
  double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
  double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
  double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
  uint16_t printCount = 0; //counter to avoid printing every 10MS sample

  if (!IMU055.begin()) {
    IMU055_connected = false;
    xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    Serial.println("IMU ...!!!... [[FAIL]]");
    xSemaphoreGive(Serial_mutex);
  } else {
    IMU055_connected = true;
    xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    Serial.println("IMU ...!!!... [[FAIL]]");
    xSemaphoreGive(Serial_mutex);
  }

  while (IMU055_connected = true) {
    unsigned long tStart = micros();

    xSemaphoreTake(IMU055_mutex, portMAX_DELAY);
    IMU055.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    IMU055.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

    // velocity of sensor in the direction it's facing
    headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

    self_status.IMU055_orientation_x = orientationData.orientation.x;
    self_status.IMU055_orientation_y = orientationData.orientation.y;
    self_status.IMU055_orientation_z = orientationData.orientation.z;

    // Data Collection Complete
    xSemaphoreGive(IMU055_mutex);
    xSemaphoreGive(IMU055_new);

    // Print Results (should be it's own task?)
    if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
      //enough iterations have passed that we can print the latest data
      xSemaphoreTake(Serial_mutex, portMAX_DELAY);
      Serial.print("Heading: ");
      xSemaphoreTake(IMU055_mutex, portMAX_DELAY);
      Serial.println(orientationData.orientation.x);
      Serial.print("Position: ");
      Serial.print(xPos);
      Serial.print(" , ");
      Serial.println(yPos);
      Serial.print("Speed: ");
      Serial.println(headingVel);
      xSemaphoreGive(IMU055_mutex);
      Serial.println("-------");
      xSemaphoreGive(Serial_mutex);
      printCount = 0;
    }
    else {
      printCount = printCount + 1;
    }
    // This seems clumsy?
    while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
    {
      taskYIELD();
    }
  }
}
#pragma endregion
#pragma region IMU085
// For SPI mode, we need a CS` pin
// #define BNO08X_CS 10
#define BNO08X_INT 9

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setReports(void) {
    Serial.println("Setting desired reports");
    if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
      Serial.println("Could not enable accelerometer");
    }
    if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
      Serial.println("Could not enable gyroscope");
    }
    if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
      Serial.println("Could not enable magnetic field calibrated");
    }
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
      Serial.println("Could not enable linear acceleration");
    }
    if (!bno08x.enableReport(SH2_GRAVITY)) {
      Serial.println("Could not enable gravity vector");
    }
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      Serial.println("Could not enable rotation vector");
    }
    if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
      Serial.println("Could not enable geomagnetic rotation vector");
    }
    if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
      Serial.println("Could not enable game rotation vector");
    }
    if (!bno08x.enableReport(SH2_STEP_COUNTER)) {
      Serial.println("Could not enable step counter");
    }
    if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER)) {
      Serial.println("Could not enable stability classifier");
    }
    if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
      Serial.println("Could not enable raw accelerometer");
    }
    if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
      Serial.println("Could not enable raw gyroscope");
    }
    if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
      Serial.println("Could not enable raw magnetometer");
    }
    if (!bno08x.enableReport(SH2_SHAKE_DETECTOR)) {
      Serial.println("Could not enable shake detector");
    }
    if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER)) {
      Serial.println("Could not enable personal activity classifier");
    }
}
void printActivity(uint8_t activity_id) {
    switch (activity_id) {
      case PAC_UNKNOWN:
        Serial.print("Unknown");
        break;
      case PAC_IN_VEHICLE:
        Serial.print("In Vehicle");
        break;
      case PAC_ON_BICYCLE:
        Serial.print("On Bicycle");
        break;
      case PAC_ON_FOOT:
        Serial.print("On Foot");
        break;
      case PAC_STILL:
        Serial.print("Still");
        break;
      case PAC_TILTING:
        Serial.print("Tilting");
        break;
      case PAC_WALKING:
        Serial.print("Walking");
        break;
      case PAC_RUNNING:
        Serial.print("Running");
        break;
      case PAC_ON_STAIRS:
        Serial.print("On Stairs");
        break;
      default:
        Serial.print("NOT LISTED");
    }
    Serial.print(" (");
    Serial.print(activity_id);
    Serial.print(")");
}

bool IMU085_connected = false;
static SemaphoreHandle_t IMU085_mutex;   // Locks IMU object
static SemaphoreHandle_t IMU085_new;     //Signals NEW IMU data

void handleIMU085(void *parameter) {
  xSemaphoreTake(Serial_mutex, portMAX_DELAY);
  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    IMU085_connected = false;
    Serial.println("Failed to find BNO08x chip");
  } else {
    IMU085_connected = true;
    Serial.println("BNO08x Found!");
    for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
      Serial.print("Part ");
      Serial.print(bno08x.prodIds.entry[n].swPartNumber);
      Serial.print(": Version :");
      Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
      Serial.print(".");
      Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
      Serial.print(".");
      Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
      Serial.print(" Build ");
      Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
    }
  }

  if (IMU085_connected) {
    setReports();
    Serial.println("IMU085 Startup /// [OK]");
  }

  xSemaphoreGive(Serial_mutex);
  
  // 'Loop'
  while (IMU085_connected) {
    vTaskDelay(10 / portTICK_PERIOD_MS);

    if (bno08x.wasReset()) {
      xSemaphoreTake(Serial_mutex, portMAX_DELAY);
      Serial.print("sensor was reset ");
      setReports();
      xSemaphoreGive(Serial_mutex);
    }

    if (!bno08x.getSensorEvent(&sensorValue)) {
      return;
    }

    xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    switch (sensorValue.sensorId) {
      case SH2_ACCELEROMETER:
        Serial.print("Accelerometer - x: ");
        Serial.print(sensorValue.un.accelerometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.accelerometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.accelerometer.z);
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        Serial.print("Gyro - x: ");
        Serial.print(sensorValue.un.gyroscope.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.gyroscope.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.gyroscope.z);
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        Serial.print("Magnetic Field - x: ");
        Serial.print(sensorValue.un.magneticField.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.magneticField.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.magneticField.z);
        break;
      case SH2_LINEAR_ACCELERATION:
        Serial.print("Linear Acceration - x: ");
        Serial.print(sensorValue.un.linearAcceleration.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.linearAcceleration.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.linearAcceleration.z);
        break;
      case SH2_GRAVITY:
        Serial.print("Gravity - x: ");
        Serial.print(sensorValue.un.gravity.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.gravity.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.gravity.z);
        break;
      case SH2_ROTATION_VECTOR:
        Serial.print("Rotation Vector - r: ");
        Serial.print(sensorValue.un.rotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.rotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.rotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.rotationVector.k);
        xSemaphoreGive(Serial_mutex);

        float qr = sensorValue.un.rotationVector.real;
        float qi = sensorValue.un.rotationVector.i;
        float qj = sensorValue.un.rotationVector.j; 
        float qk = sensorValue.un.rotationVector.k;

        float sqr = sq(qr);
        float sqi = sq(qi);
        float sqj = sq(qj);
        float sqk = sq(qk);

        float yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
        float pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
        float roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
        
        // Convert to degrees
        yaw *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        roll *= RAD_TO_DEG;

        // Assign to status object
        xSemaphoreTake(IMU085_mutex, portMAX_DELAY);                          
        self_status.IMU085_rotationVector_real = sensorValue.un.rotationVector.real;
        self_status.IMU085_rotationVector_i = sensorValue.un.rotationVector.i;                          
        self_status.IMU085_rotationVector_j = sensorValue.un.rotationVector.j;   
        self_status.IMU085_rotationVector_k = sensorValue.un.rotationVector.k;
        self_status.IMU085_rotationVector_yaw = yaw;
        self_status.IMU085_rotationVector_pitch = pitch;
        self_status.IMU085_rotationVector_roll  = roll;
        xSemaphoreGive(IMU085_mutex);   
        xSemaphoreGive(IMU085_new);   
        break;

      case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        Serial.print("Geo-Magnetic Rotation Vector - r: ");
        Serial.print(sensorValue.un.geoMagRotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.geoMagRotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.geoMagRotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.geoMagRotationVector.k);
        break;

      case SH2_GAME_ROTATION_VECTOR:
        Serial.print("Game Rotation Vector - r: ");
        Serial.print(sensorValue.un.gameRotationVector.real);
        Serial.print(" i: ");
        Serial.print(sensorValue.un.gameRotationVector.i);
        Serial.print(" j: ");
        Serial.print(sensorValue.un.gameRotationVector.j);
        Serial.print(" k: ");
        Serial.println(sensorValue.un.gameRotationVector.k);
        break;

      case SH2_STEP_COUNTER:
        Serial.print("Step Counter - steps: ");
        Serial.print(sensorValue.un.stepCounter.steps);
        Serial.print(" latency: ");
        Serial.println(sensorValue.un.stepCounter.latency);
        break;

      case SH2_STABILITY_CLASSIFIER: {
          Serial.print("Stability Classification: ");
          sh2_StabilityClassifier_t stability = sensorValue.un.stabilityClassifier;
          switch (stability.classification) {
            case STABILITY_CLASSIFIER_UNKNOWN:
              Serial.println("Unknown");
              break;
            case STABILITY_CLASSIFIER_ON_TABLE:
              Serial.println("On Table");
              break;
            case STABILITY_CLASSIFIER_STATIONARY:
              Serial.println("Stationary");
              break;
            case STABILITY_CLASSIFIER_STABLE:
              Serial.println("Stable");
              break;
            case STABILITY_CLASSIFIER_MOTION:
              Serial.println("In Motion");
              break;
          }
          break;
        }

      case SH2_RAW_ACCELEROMETER:
        Serial.print("Raw Accelerometer - x: ");
        Serial.print(sensorValue.un.rawAccelerometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawAccelerometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawAccelerometer.z);
        break;
      case SH2_RAW_GYROSCOPE:
        Serial.print("Raw Gyro - x: ");
        Serial.print(sensorValue.un.rawGyroscope.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawGyroscope.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawGyroscope.z);
        break;
      case SH2_RAW_MAGNETOMETER:
        Serial.print("Raw Magnetic Field - x: ");
        Serial.print(sensorValue.un.rawMagnetometer.x);
        Serial.print(" y: ");
        Serial.print(sensorValue.un.rawMagnetometer.y);
        Serial.print(" z: ");
        Serial.println(sensorValue.un.rawMagnetometer.z);
        break;

      case SH2_SHAKE_DETECTOR: {
          Serial.print("Shake Detector - shake detected on axis: ");
          sh2_ShakeDetector_t detection = sensorValue.un.shakeDetector;
          switch (detection.shake) {
            case SHAKE_X:
              Serial.println("X");
              break;
            case SHAKE_Y:
              Serial.println("Y");
              break;
            case SHAKE_Z:
              Serial.println("Z");
              break;
            default:
              Serial.println("None");
              break;
          }
        }

      case SH2_PERSONAL_ACTIVITY_CLASSIFIER: {
          sh2_PersonalActivityClassifier_t activity = sensorValue.un.personalActivityClassifier;
          uint8_t page_num = activity.page;
          Serial.print("Activity classification - Most likely: ");
          printActivity(activity.mostLikelyState);
          Serial.println("");

          Serial.println("Confidences:");
          // if PAC_OPTION_COUNT is ever > 10, we'll need to
          // care about page
          for (uint8_t i = 0; i < PAC_OPTION_COUNT; i++) {
            Serial.print("\t");
            printActivity(i);
            Serial.print(": ");
            Serial.println(activity.confidence[i]);
          }
        }
    }
    xSemaphoreGive(Serial_mutex);
  }
}
#pragma endregion

#pragma region Display
// Display
TFT_eSPI tft = TFT_eSPI();
// Screen Buffers
#define STATIC_LINES 5
#define EXTRA_PX 7
//   ffDisplay.terminalScreen.setCursor(
//       0, 135 - ffDisplay.terminalScreen.fontHeight());
#define NEWLINE_X 0
#define NEWLINE_Y 135 - 16
TFT_eSprite MSD_screen = TFT_eSprite(&tft);

static SemaphoreHandle_t TFT_mutex;   // Locks IMU object
static SemaphoreHandle_t TFT_new;     //Signals NEW IMU data

void updateTFT(void *parameter) {
  while (1) {
    // Line 1 (Time)
    MSD_screen.setCursor(0, 0);
    MSD_screen.setTextColor(TFT_DARKGREY, TFT_BLACK);
    MSD_screen.print(F("TIME // [UNDEFINED]"));

    // Line 2 (Space)
    MSD_screen.setCursor(0, 16);
    if (!GPS_connected) MSD_screen.setTextColor(TFT_DARKGREY, TFT_BLACK);
    if (GPS_connected) MSD_screen.setTextColor(TFT_ORANGE, TFT_BLACK);
    xSemaphoreTake(GPS_new, portMAX_DELAY);
    xSemaphoreTake(GPS_mutex, portMAX_DELAY);
    xSemaphoreTake(TFT_mutex, portMAX_DELAY);
    if (!GPS.fix) MSD_screen.print(F("SPACE // [INVALID]"));
    if (GPS.fix) {
      MSD_screen.print(F("SPACE // "));
      MSD_screen.print(self_status.latitude, 7);
      MSD_screen.print(F(", "));
      MSD_screen.print(self_status.longitude, 7);
    }
    xSemaphoreGive(TFT_mutex);
    xSemaphoreGive(GPS_mutex);

    // Line 3 (POV)
    xSemaphoreTake(IMU085_new, portMAX_DELAY);
    xSemaphoreTake(IMU085_mutex, portMAX_DELAY);
    MSD_screen.setCursor(0, 32);
    MSD_screen.setTextColor(TFT_ORANGE, TFT_BLACK);
    MSD_screen.print(F("YPR // "));
    MSD_screen.print(self_status.IMU085_rotationVector_yaw);
    MSD_screen.print(F(", "));
    MSD_screen.print(self_status.IMU085_rotationVector_pitch);
    MSD_screen.print(F(", "));
    MSD_screen.print(self_status.IMU085_rotationVector_roll);
    xSemaphoreGive(IMU085_mutex);
    xSemaphoreGive(TFT_mutex);
    


    MSD_screen.pushSprite(0, 0);
    

    vTaskDelay(16 / portTICK_PERIOD_MS); // 16 ms should be ~60 fps
  }
}
#pragma endregion

void setup() {
  // Serial
  Serial_mutex = xSemaphoreCreateMutex();
  xSemaphoreTake(Serial_mutex, portMAX_DELAY);
  Serial.begin(115200);
  Serial.println("Startup...");
  xSemaphoreGive(Serial_mutex);

  // OTA Task
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    handleOTA,    // Function to be called
    "Handle OTA", // Name of task
    2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,         // Parameter to pass to function
    1,            // Task priority (0 to configMAX_PRIORITIES - 1)
    NULL,         // Task handle
    app_cpu);     // Run on one core for demo purposes (ESP32 only)

  // Configure LED
  pinMode(led_pin, OUTPUT);
  // LED Task
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    toggleLED,    // Function to be called
    "Toggle LED", // Name of task
    1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,         // Parameter to pass to function
    1,            // Task priority (0 to configMAX_PRIORITIES - 1)
    NULL,         // Task handle
    app_cpu);     // Run on one core for demo purposes (ESP32 only)

  // Setup GPS
  if (!GPS.begin(9600)) {
    xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    Serial.println("GPS ...!!!... [[FAIL]]");
    xSemaphoreGive(Serial_mutex);
    GPS_connected = false;
  } else {
    xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    Serial.println("GPS ...///... [OK]");
    xSemaphoreGive(Serial_mutex);
    GPS_connected = true;
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 second
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 10 Seconds
    //GPS.sendCommand(PGCMD_ANTENNA); //Request antennae status as well?
  }

  // FFStatus
  status_mutex = xSemaphoreCreateMutex();

  // GPS Tasks
  GPS_mutex = xSemaphoreCreateMutex();
  GPS_new = xSemaphoreCreateBinary();
  //xSemaphoreTake(GPS_new, portMAX_DELAY);

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    updateGPS,    // Function to be called
    "Update GPS", // Name of task
    2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,         // Parameter to pass to function
    1,            // Task priority (0 to configMAX_PRIORITIES - 1)
    NULL,         // Task handle
    app_cpu);     // Run on one core for demo purposes (ESP32 only)

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    printGPS,    // Function to be called
    "Print GPS", // Name of task
    1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,         // Parameter to pass to function
    1,            // Task priority (0 to configMAX_PRIORITIES - 1)
    NULL,         // Task handle
    app_cpu);     // Run on one core for demo purposes (ESP32 only)

  // Setup IMU


  // IMU Tasks
  IMU055_mutex = xSemaphoreCreateMutex();
  IMU055_new = xSemaphoreCreateBinary();

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    handleIMU055,    // Function to be called
    "Handle IMU055", // Name of task
    2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,         // Parameter to pass to function
    1,            // Task priority (0 to configMAX_PRIORITIES - 1)
    NULL,         // Task handle
    app_cpu);     // Run on one core for demo purposes (ESP32 only)

  // TFT Display Setup
  pinMode(TFT_BL, OUTPUT);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // Setup "Terminal" Screen/Spite
  MSD_screen.setRotation(1);
  MSD_screen.createSprite(240, 135);
  MSD_screen.fillSprite(TFT_BLACK);
  MSD_screen.setTextFont(2);

  // TFT Tasks
  TFT_mutex = xSemaphoreCreateMutex();
  TFT_new = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(    // Use xTaskCreate() in vanilla FreeRTOS
    updateTFT,    // Function to be called
    "Update TFT", // Name of task
    10000,         // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,         // Parameter to pass to function
    1,            // Task priority (0 to configMAX_PRIORITIES - 1)
    NULL,         // Task handle
    app_cpu);     // Run on one core for demo purposes (ESP32 only)


  xSemaphoreTake(Serial_mutex, portMAX_DELAY);
  Serial.println("All tasks created");
  xSemaphoreGive(Serial_mutex);
}



void loop() {
  // Do nothing but allow yielding to lower-priority tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // setup() and loop() run in their own task with priority 1 in core 1
  // on ESP32
}
