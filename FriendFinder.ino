#include "FriendFinder.h"
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include "AF_240x135_ST7789.h" // Local config header
#define USER_SETUP_LOADED      // forces use of local config header
#include <TFT_eSPI.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BNO08x.h>

// FreeRTOS
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// FF Data
ffStatus self_status;
static SemaphoreHandle_t status_mutex; // Locks status_object do I need this?

#pragma region Radio

// RFM95W LORA Module
#define MY_ADDRESS 2
#define THEIR_ADDRESS 1
#define RFM95_CS 33
#define RFM95_RST 14
#define RFM95_INT 21
RH_RF95 driver(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(driver, MY_ADDRESS);

void resetLORA()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  //vTaskDelay(100 / portTICK_PERIOD_MS);
  digitalWrite(RFM95_RST, LOW);
  //vTaskDelay(10 / portTICK_PERIOD_MS);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  //vTaskDelay(10 / portTICK_PERIOD_MS);
  delay(10);
  //vTaskDelay(1000 / portTICK_PERIOD_MS);
  delay(1000);
}

static SemaphoreHandle_t LORA_mutex;
TaskHandle_t task_LORA;

void handleLORA(void *parameter)
{
  while (1)
  {
    // Radio not connected - try to start
    if (!self_status.LORA_connected)
    {
      Serial.println("Radio /// [ RESET ... ]");
      resetLORA(); // hardware reset

      if (!manager.init())
      {
        self_status.LORA_connected = false;
        Serial.println("Radio /// [ .. FAIL .. ] { MANAGER }");
      }
      else
      {
        if (!driver.setFrequency(915.0))
        {
          self_status.LORA_connected = false;
          Serial.println("Radio /// [ .. FAIL .. ] { FREQ SET }");
        }
        else
        {
          // You can set transmitter powers from 5 to 23 dBm:
          driver.setTxPower(23, false);
          driver.setCADTimeout(5000);
          self_status.LORA_connected = true;
          Serial.println("Radio /// [ OK ]");
        }
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    // Radio is connected
    if (self_status.LORA_connected)
    {
      Serial.println("Radio /// [ (at your command) ]");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

#pragma endregion
#pragma region Serial Output
// Serial Terminal Output
static SemaphoreHandle_t Serial_mutex; // Locks Serial object
/* To do:
  Comprehensive task for handling serial outputs
    - use queues so tasks can hand off data and continue without
      blocking, and still everything gets printed.
    - task can be switched on and off to only enable when needed
*/

#pragma endregion

#pragma region OTA Update
// OTA Update
#define OTA_STARTUP_TIME_MS 30000 // How long OTA turns on for at startup (used by controller)
TaskHandle_t task_OTA;

void handleOTA(void *parameter)
{
  const char *ssid = "***REMOVED***";
  const char *password = "***REMOVED***";

  while (1)
  {
    // Suspend self if should be off
    //if (!self_status.OTA_enable) vTaskSuspend( NULL );
    // Try to Connect if not connected
    if (!WiFi.isConnected())
    {
      self_status.xTicks_When_Wifi_Disconnected = xTaskGetTickCount(); // now
      while (!WiFi.isConnected())
      {
        WiFi.mode(WIFI_STA);
        Serial.println("WiFi /// [Connecting...]");
        WiFi.begin(ssid, password);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        while (WiFi.waitForConnectResult() != WL_CONNECTED)
        {
          Serial.println("WiFi /// [..retry connection..]");
          WiFi.reconnect();
          vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
      }
      self_status.xTicks_When_Wifi_Disconnected = 0;
      ArduinoOTA.setHostname("FFProto");
      Serial.println("WiFi /// [..CONNECTED..]");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());

      // Start OTA
      ArduinoOTA
          .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
              type = "sketch";
            else // U_SPIFFS
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
    }

    // Handle OTA if connected
    if (WiFi.isConnected())
    {
      ArduinoOTA.handle();
      taskYIELD();
    }
  } // end loop
}
#pragma endregion

#pragma region GPS
// GPS
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false               // Echo GPS to Serial terminal?
static SemaphoreHandle_t GPS_mutex; // Locks GPS object
static SemaphoreHandle_t GPS_new;   //Signals NEW GPS data

void updateGPS(void *parameter)
{
  while (1)
  {
    char c = GPS.read();
    if (GPS.newNMEAreceived())
    {
      if (GPS.parse(GPS.lastNMEA())) // sets newNMEAreceived() = false
      {
        // one byte date needs no semaphore
        self_status.GPS_fix = GPS.fix;
        self_status.fixquality = GPS.fixquality;
        self_status.fixquality_3d = GPS.fixquality_3d;
        self_status.satellites = GPS.satellites;
        if (xSemaphoreTake(GPS_mutex, 0))
        {
          self_status.latitude_fixed = GPS.latitude_fixed;
          self_status.longitude_fixed = GPS.longitude_fixed;
          self_status.latitude = GPS.latitude_fixed / 10000000.0;
          self_status.longitude = GPS.longitude_fixed / 10000000.0;
          xSemaphoreGive(GPS_mutex);
        }
        xSemaphoreGive(GPS_new);
      }
    }
  }
}


#pragma endregion
#pragma region LED / Heartbeat
// LED
static const int led_pin = LED_BUILTIN;
TaskHandle_t task_LED;
void toggleLED(void *parameter)
{
  while (1)
  {
    // Analog write implementation is weird see:
    // https://github.com/espressif/arduino-esp32/issues/4

    // digitalWrite(led_pin, HIGH);
    // vTaskDelay(250 / portTICK_PERIOD_MS);
    // digitalWrite(led_pin, LOW);
    vTaskDelay(1750 / portTICK_PERIOD_MS);

    if (xSemaphoreTake(Serial_mutex, 0))
    {
      Serial.println("Heartbeat <3");
      Serial.println("[IMU_085 | (..reports) | GPS | Radio] 1=Good 0 = Bad");
      Serial.print(self_status.IMU085_connected);
      Serial.print(self_status.IMU085_reports_set);
      Serial.print(self_status.GPS_connected);
      Serial.println(self_status.LORA_connected);
      xSemaphoreGive(Serial_mutex);
    }
    else
    {
      Serial.println("** Serial Locked **");
    }
  }
}
#pragma endregion

#pragma region IMU085
#define BNO08X_INT 9
#define BNO08X_RESET -1 // set to -1 for I2C or UART

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

bool setReports(void)
{
  bool reports_set = true;
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER))
  {
    reports_set = false; // Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
  {
    reports_set = false; // Serial.println("Could not enable gyroscope");
  }
  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED))
  {
    reports_set = false; // Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION))
  {
    reports_set = false; // Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GRAVITY))
  {
    reports_set = false; // Serial.println("Could not enable gravity vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
  {
    reports_set = false; // Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR))
  {
    reports_set = false; // Serial.println("Could not enable geomagnetic rotation vector");
  }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR))
  {
    reports_set = false; // Serial.println("Could not enable game rotation vector");
  }
  if (!bno08x.enableReport(SH2_STEP_COUNTER))
  {
    reports_set = false; // Serial.println("Could not enable step counter");
  }
  if (!bno08x.enableReport(SH2_STABILITY_CLASSIFIER))
  {
    reports_set = false; // Serial.println("Could not enable stability classifier");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER))
  {
    reports_set = false; // Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE))
  {
    reports_set = false; // Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER))
  {
    reports_set = false; // Serial.println("Could not enable raw magnetometer");
  }
  if (!bno08x.enableReport(SH2_SHAKE_DETECTOR))
  {
    reports_set = false; // Serial.println("Could not enable shake detector");
  }
  if (!bno08x.enableReport(SH2_PERSONAL_ACTIVITY_CLASSIFIER))
  {
    reports_set = false; // Serial.println("Could not enable personal activity classifier");
  }
  return reports_set;
}

static SemaphoreHandle_t IMU085_mutex; // Locks IMU object
static SemaphoreHandle_t IMU085_new;   //Signals NEW IMU data

void handleIMU085(void *parameter)
{
  // I don't know what the sample rate should be so just go fast for now
  uint16_t BNO085_samplerate_delay_ms = 1;                                       // how long between readings (dbl check w/ hardware)
  TickType_t xLastWakeTime;                                                      // object to store execution timing
  const TickType_t xFrequency = BNO085_samplerate_delay_ms / portTICK_PERIOD_MS; // how many ticks between updates

  while (1)
  {
    // Startup / Reconnect
    if (!self_status.IMU085_connected) // not connected
    {
      // Try to initialize!
      if (!bno08x.begin_I2C())
      {
        Serial.println("IMU_085 NOT Connected");
        self_status.IMU085_connected = false;
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
      else
      {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (!setReports())
        {
          self_status.IMU085_reports_set = false;
          Serial.println("IMU_085 Reports Not Set");
        }
        else
        {
          self_status.IMU085_reports_set = true;
          Serial.println("IMU_085 Reports Set");
        }
        Serial.println("IMU_085 Connected");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        self_status.IMU085_connected = true;
      }
    }
    else // is connected
    {
      if (bno08x.wasReset())
      {
        // Serial.println("IMU_085 Was Reset");
        // self_status.IMU085_reports_set = setReports();
      }
      xLastWakeTime = xTaskGetTickCount();
      if (!bno08x.getSensorEvent(&sensorValue))
      {
        // Did not get a value, wait one cycle
      }
      else
      {
        switch (sensorValue.sensorId)
        {
        case SH2_ACCELEROMETER:
          // Serial.print("Accelerometer - x: ");
          // Serial.print(sensorValue.un.accelerometer.x);
          // Serial.print(" y: ");
          // Serial.print(sensorValue.un.accelerometer.y);
          // Serial.print(" z: ");
          // Serial.println(sensorValue.un.accelerometer.z);
          break;
        case SH2_GYROSCOPE_CALIBRATED:
          // Serial.print("Gyro - x: ");
          // Serial.print(sensorValue.un.gyroscope.x);
          // Serial.print(" y: ");
          // Serial.print(sensorValue.un.gyroscope.y);
          // Serial.print(" z: ");
          // Serial.println(sensorValue.un.gyroscope.z);
          break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
          // Serial.print("Magnetic Field - x: ");
          // Serial.print(sensorValue.un.magneticField.x);
          // Serial.print(" y: ");
          // Serial.print(sensorValue.un.magneticField.y);
          // Serial.print(" z: ");
          // Serial.println(sensorValue.un.magneticField.z);
          break;
        case SH2_LINEAR_ACCELERATION:
          // Serial.print("Linear Acceration - x: ");
          // Serial.print(sensorValue.un.linearAcceleration.x);
          // Serial.print(" y: ");
          // Serial.print(sensorValue.un.linearAcceleration.y);
          // Serial.print(" z: ");
          // Serial.println(sensorValue.un.linearAcceleration.z);
          break;
        case SH2_GRAVITY:
          // Serial.print("Gravity - x: ");
          // Serial.print(sensorValue.un.gravity.x);
          // Serial.print(" y: ");
          // Serial.print(sensorValue.un.gravity.y);
          // Serial.print(" z: ");
          // Serial.println(sensorValue.un.gravity.z);
          break;
        case SH2_ROTATION_VECTOR:
        {
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
          if (xSemaphoreTake(IMU085_mutex, (TickType_t)10 / portTICK_PERIOD_MS) == pdTRUE)
          {
            self_status.IMU085_rotationVector_real = sensorValue.un.rotationVector.real;
            self_status.IMU085_rotationVector_i = sensorValue.un.rotationVector.i;
            self_status.IMU085_rotationVector_j = sensorValue.un.rotationVector.j;
            self_status.IMU085_rotationVector_k = sensorValue.un.rotationVector.k;
            self_status.IMU085_rotationVector_yaw = yaw;
            self_status.IMU085_rotationVector_pitch = pitch;
            self_status.IMU085_rotationVector_roll = roll;
            xSemaphoreGive(IMU085_mutex);
            xSemaphoreGive(IMU085_new);
          }
          else
          {
            // Couldn't store data should I do something?
          }
          break;
        }
        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
          // Serial.print("Geo-Magnetic Rotation Vector - r: ");
          // Serial.print(sensorValue.un.geoMagRotationVector.real);
          // Serial.print(" i: ");
          // Serial.print(sensorValue.un.geoMagRotationVector.i);
          // Serial.print(" j: ");
          // Serial.print(sensorValue.un.geoMagRotationVector.j);
          // Serial.print(" k: ");
          // Serial.println(sensorValue.un.geoMagRotationVector.k);
          break;

        case SH2_GAME_ROTATION_VECTOR:
          // Serial.print("Game Rotation Vector - r: ");
          // Serial.print(sensorValue.un.gameRotationVector.real);
          // Serial.print(" i: ");
          // Serial.print(sensorValue.un.gameRotationVector.i);
          // Serial.print(" j: ");
          // Serial.print(sensorValue.un.gameRotationVector.j);
          // Serial.print(" k: ");
          // Serial.println(sensorValue.un.gameRotationVector.k);
          break;

        case SH2_STEP_COUNTER:
          // Serial.print("Step Counter - steps: ");
          // Serial.print(sensorValue.un.stepCounter.steps);
          // Serial.print(" latency: ");
          // Serial.println(sensorValue.un.stepCounter.latency);
          break;

        case SH2_STABILITY_CLASSIFIER:
        {
          // Serial.print("Stability Classification: ");
          // sh2_StabilityClassifier_t stability = sensorValue.un.stabilityClassifier;
          // switch (stability.classification)
          // {
          // case STABILITY_CLASSIFIER_UNKNOWN:
          //   Serial.println("Unknown");
          //   break;
          // case STABILITY_CLASSIFIER_ON_TABLE:
          //   Serial.println("On Table");
          //   break;
          // case STABILITY_CLASSIFIER_STATIONARY:
          //   Serial.println("Stationary");
          //   break;
          // case STABILITY_CLASSIFIER_STABLE:
          //   Serial.println("Stable");
          //   break;
          // case STABILITY_CLASSIFIER_MOTION:
          //   Serial.println("In Motion");
          //   break;
          // }
          break;
        }
        case SH2_RAW_ACCELEROMETER:
          // Serial.print("Raw Accelerometer - x: ");
          // Serial.print(sensorValue.un.rawAccelerometer.x);
          // Serial.print(" y: ");
          // Serial.print(sensorValue.un.rawAccelerometer.y);
          // Serial.print(" z: ");
          // Serial.println(sensorValue.un.rawAccelerometer.z);
          break;
        case SH2_RAW_GYROSCOPE:
          // Serial.print("Raw Gyro - x: ");
          // Serial.print(sensorValue.un.rawGyroscope.x);
          // Serial.print(" y: ");
          // Serial.print(sensorValue.un.rawGyroscope.y);
          // Serial.print(" z: ");
          // Serial.println(sensorValue.un.rawGyroscope.z);
          break;
        case SH2_RAW_MAGNETOMETER:
          // Serial.print("Raw Magnetic Field - x: ");
          // Serial.print(sensorValue.un.rawMagnetometer.x);
          // Serial.print(" y: ");
          // Serial.print(sensorValue.un.rawMagnetometer.y);
          // Serial.print(" z: ");
          // Serial.println(sensorValue.un.rawMagnetometer.z);
          break;

        case SH2_SHAKE_DETECTOR:
        {
          // Serial.print("Shake Detector - shake detected on axis: ");
          // sh2_ShakeDetector_t detection = sensorValue.un.shakeDetector;
          // switch (detection.shake)
          // {
          // case SHAKE_X:
          //   Serial.println("X");
          //   break;
          // case SHAKE_Y:
          //   Serial.println("Y");
          //   break;
          // case SHAKE_Z:
          //   Serial.println("Z");
          //   break;
          // default:
          //   Serial.println("None");
          //   break;
          // }
        }
        case SH2_PERSONAL_ACTIVITY_CLASSIFIER:
        {
          // sh2_PersonalActivityClassifier_t activity = sensorValue.un.personalActivityClassifier;
          // uint8_t page_num = activity.page;
          // Serial.print("Activity classification - Most likely: ");
          // printActivity(activity.mostLikelyState);
          // Serial.println("");

          // Serial.println("Confidences:");
          // // if PAC_OPTION_COUNT is ever > 10, we'll need to
          // // care about page
          // for (uint8_t i = 0; i < PAC_OPTION_COUNT; i++)
          // {
          //   Serial.print("\t");
          //   printActivity(i);
          //   Serial.print(": ");
          //   Serial.println(activity.confidence[i]);
          // }
        }
        }
      }
      // Wait until next cycle
      //vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
  }
}
#pragma endregion

#pragma region Display
// Display
#define TFT_STARTUP_TIME_MS 10000 // How long TFT turns on for at startup (used by controller)
TFT_eSPI tft = TFT_eSPI();
// Screen Buffers
#define STATIC_LINES 5
#define EXTRA_PX 7
//   ffDisplay.terminalScreen.setCursor(
//       0, 135 - ffDisplay.terminalScreen.fontHeight());
#define NEWLINE_X 0
#define NEWLINE_Y 135 - 16
TFT_eSprite MSD_screen = TFT_eSprite(&tft);

static SemaphoreHandle_t TFT_mutex; // Locks IMU object
static SemaphoreHandle_t TFT_new;   //Signals NEW IMU data

void lineBlank()
{
  MSD_screen.fillRect(MSD_screen.getCursorX(),
                      MSD_screen.getCursorY(),
                      MSD_screen.width(),
                      MSD_screen.fontHeight(),
                      MSD_screen.textbgcolor);
}

// Task
TaskHandle_t task_TFT;
void handleTFT(void *parameter)
{
  Serial.println("Display Startup");

  // TFT Display Setup

  pinMode(TFT_BL, OUTPUT);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_ORANGE);

  // Setup "Master System Display" Screen/Spite
  MSD_screen.setRotation(1);
  MSD_screen.createSprite(240, 135);
  MSD_screen.fillSprite(TFT_BLACK);
  MSD_screen.setTextFont(2);

  while (1)
  {
    // Line 1 (Time)
    MSD_screen.setCursor(0, 0);
    MSD_screen.setTextColor(TFT_DARKGREY, TFT_BLACK);
    MSD_screen.print(F("TIME // [ UNDEFINED ]"));
    MSD_screen.pushSprite(0, 0);

    // Line 2 (Space)
    MSD_screen.setCursor(0, 16);
    if (!self_status.GPS_connected)
    { // GPS Not Connected
      MSD_screen.setTextColor(FFPAL_PURPLE, TFT_BLACK);
      lineBlank();
      MSD_screen.print(F("SPACE // [[ FAIL ]]"));
    }
    else
    {
      if (!self_status.GPS_fix)
      { // ...but no fix
        if (self_status.GPS_naive)
        { // and GPS has never had a fix
          MSD_screen.setTextColor(FFPAL_BLUE, TFT_BLACK);
          lineBlank();
          MSD_screen.print(F("SPACE // [ INVALID ] { "));
          MSD_screen.print(self_status.satellites);
          MSD_screen.print(F(" }"));
        }
        else
        { // GPS has had a fix before
          MSD_screen.setTextColor(FFPAL_PURPLE, TFT_BLACK);
          lineBlank();
          MSD_screen.print(F("SPACE // [ LOST ] { "));
          MSD_screen.print(self_status.satellites);
          MSD_screen.print(F(" }"));
        }
      }
      else
      { // GPS Connected with Fix
        // Try to store data for 1 ms
        if (xSemaphoreTake(GPS_mutex, 0))
        {
          if (self_status.GPS_naive)
            self_status.GPS_naive = false; // no longer naive
          float lat = self_status.latitude;
          float lon = self_status.longitude;
          xSemaphoreGive(GPS_mutex);
          MSD_screen.setTextColor(TFT_ORANGE, TFT_BLACK);
          lineBlank();
          MSD_screen.print(F("SPACE // "));
          MSD_screen.print(lat, 7);
          MSD_screen.print(F(", "));
          MSD_screen.print(lon, 7);
        }
        else
        { // Cant retrieve sempahore, probably should just pass() but I wanna see if it happens
          MSD_screen.setTextColor(TFT_RED, TFT_BLACK);
          MSD_screen.print(F("SPACE // [?? Blocked ??]"));
        }
      }
    }
    MSD_screen.pushSprite(0, 0);

    // Line 3 (IMU) [BNO085]
    MSD_screen.setCursor(0, 32);
    if (!self_status.IMU085_connected)
    {
      MSD_screen.setTextColor(TFT_DARKGREY, TFT_BLACK);
      lineBlank();
      MSD_screen.print(F("YPR // ... [[NULL]] ..."));
      MSD_screen.pushSprite(0, 0);
    }
    else
    {
      //if (xSemaphoreTake(IMU085_new, 0))
      if (true)
      {
        float yaw, pitch, roll;
        if (xSemaphoreTake(IMU085_mutex, 0))
        {
          yaw = self_status.IMU085_rotationVector_yaw;
          pitch = self_status.IMU085_rotationVector_pitch;
          roll = self_status.IMU085_rotationVector_roll;
          xSemaphoreGive(IMU085_mutex);
          MSD_screen.setTextColor(TFT_ORANGE, TFT_BLACK);
          lineBlank();
          MSD_screen.print(F("YPR // "));
          MSD_screen.print(yaw);
          MSD_screen.print(F(", "));
          MSD_screen.print(pitch);
          MSD_screen.print(F(", "));
          MSD_screen.print(roll);
          MSD_screen.pushSprite(0, 0);
        }
      }
    }

    // Line 4 (Button/Knob/ MODE)
    MSD_screen.setCursor(0, 48);
    lineBlank();
    MSD_screen.setTextColor(TFT_ORANGE, TFT_BLACK);
    MSD_screen.print(F("UI // ["));
    MSD_screen.print(self_status.knob_V);
    MSD_screen.print(F("|"));
    MSD_screen.print(self_status.knob_mode);
    MSD_screen.print(F("]"));
    MSD_screen.pushSprite(0, 0);

    // Line 5 OTA/Wifi
    MSD_screen.setCursor(0, 64);
    lineBlank();
    // If OTA task is suspended...
    if (eTaskGetState(task_OTA) == eSuspended)
    {
      // Did it time out?
      if (self_status.OTA_timeout)
      {
        MSD_screen.setTextColor(TFT_PURPLE, TFT_BLACK);
        MSD_screen.print(F("OTA // [[ Timeout ]]"));
      }
      else
      // If its off but not timeout
      {
        MSD_screen.setTextColor(FFPAL_ORANGE, TFT_BLACK);
        MSD_screen.print(F("OTA // [[ SUSPENDED ]]"));
      }
    }
    else
    {
      // If OTA Task is Running...
      // If Wifi is connected
      if (WiFi.isConnected())
      {
        if (millis() < OTA_STARTUP_TIME_MS) MSD_screen.setTextColor(FFPAL_ORANGE, TFT_BLACK);
        else MSD_screen.setTextColor(FFPAL_GREEN, TFT_BLACK);
        MSD_screen.print(F("OTA // "));
        MSD_screen.print(WiFi.localIP());
      }
      else
      // wifi not connected
      {
        MSD_screen.setTextColor(FFPAL_BLUE, TFT_BLACK);
        MSD_screen.print(F("OTA // ..CONNECTING.."));
      }
    }
    MSD_screen.pushSprite(0, 0);

    // Line 6 LORA Radio
    MSD_screen.setCursor(0, 80);
    // Radio Not Connected
    if (!self_status.LORA_connected)
    {
      lineBlank();
      MSD_screen.setTextColor(TFT_DARKGREY, TFT_BLACK);
      MSD_screen.print(F("RADIO // [[ .. OFFLINE ..  ]]"));
      MSD_screen.pushSprite(0, 0);
    }

    // Radio is Connected
    if (self_status.LORA_connected)
    {
      lineBlank();
      MSD_screen.setTextColor(TFT_ORANGE, TFT_BLACK);
      MSD_screen.print(F("RADIO // [ READY ]"));
      MSD_screen.pushSprite(0, 0);
    }
  }
}

#pragma endregion

#pragma region Knob
#define KNOB_PIN 39   // Labelled "A3" on esp32 feather
#define KNOB_MAX 4095 // (12-bit ADC has this as max)
#define KNOB_MODES 3  // How many sections to break it into?

void handleKnob(void *parameter)
{
  int knob_V;
  int knob_mode;
  while (1)
  {
    knob_V = analogRead(KNOB_PIN);
    knob_mode = map(knob_V, 0, 4095, 0, KNOB_MODES - 1);
    self_status.knob_V = knob_V;
    self_status.knob_mode = knob_mode;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
#pragma endregion

#pragma region Controller // Controls power



void handleControl(void *parameter)
{
  while (1)
  {
    // How often to check?
    vTaskDelay(20 / portTICK_PERIOD_MS);
    // Check if Display should be on
    // On reset display should come on for a few seconds
    if (millis() <= TFT_STARTUP_TIME_MS)
    {
      self_status.display_enable = true;
    }
    else
    {
      if (self_status.knob_mode == 2)
        self_status.display_enable = false;
      if (self_status.knob_mode != 2)
        self_status.display_enable = true;
    }

    // TFT is running but should be shut off
    if (eTaskGetState(task_TFT) != eSuspended && self_status.display_enable == false)
    {
      Serial.println("knob turns display off");
      vTaskSuspend(task_TFT);    // Suspend the TFT_task
      tft.fillScreen(TFT_BLACK); // Blank Screen
      digitalWrite(TFT_BL, LOW); //Backlight Off
    }

    // TFT is off but should be on
    if (eTaskGetState(task_TFT) == eSuspended && self_status.display_enable == true)
    {
      Serial.println("knob turns display on");
      tft.fillScreen(TFT_BLACK);  // Blank Screen
      digitalWrite(TFT_BL, HIGH); //Backlight On
      vTaskResume(task_TFT);      // Suspend the TFT_task
    }

    // Check if LED should be on
    // On reset display should come on for a few seconds

    // Do I need to turn the LED?
    if (self_status.knob_mode >= 2 && self_status.LED_enable)
    {
      self_status.LED_enable = false;
      vTaskSuspend(task_LED);     // Suspend the TFT_task
      digitalWrite(led_pin, LOW); //LED Off
    }
    // Do I need to turn the display on?
    if (self_status.knob_mode < 2 && !self_status.LED_enable)
    {
      vTaskResume(task_LED); // Resume the TFT_task
      self_status.LED_enable = true;
    }

    // Handle OTA
    if (self_status.knob_mode == 0)
      self_status.OTA_enable = true;
    if (self_status.knob_mode != 0)
      self_status.OTA_enable = false;
    // OTA should come on for 30s at reset for safety
    if (millis() < OTA_STARTUP_TIME_MS)
    {
      self_status.OTA_enable = true;
      self_status.OTA_timeout = false;
    }

    // OTA should disable if cant connect for 60 seconds
    //  Task is running as it should
    if (eTaskGetState(task_OTA) != eSuspended && self_status.OTA_enable == true)
    {
      // but wifi is not connected
      if (!WiFi.isConnected())
        // and its been disconnected for > 60 seconds
        if ((xTaskGetTickCount() - self_status.xTicks_When_Wifi_Disconnected) > pdMS_TO_TICKS(60000))
        {
          // Disable it (or rather set it up to be disabled)
          self_status.OTA_enable = false;
          self_status.OTA_timeout = true;
        }
    }

    // OTA Task is running but should be stopped
    if (eTaskGetState(task_OTA) != eSuspended && self_status.OTA_enable == false)
    {
      vTaskSuspend(task_OTA);      // Suspend the OTA task
      WiFi.disconnect(true, true); // Disconnect the wifi
      //WiFi.mode(WIFI_OFF);         // Turn off the radio
    }

    // OTA Task is suspended but should be resumed
    if (eTaskGetState(task_OTA) == eSuspended && self_status.OTA_enable == true)
    {
      self_status.xTicks_When_Wifi_Disconnected = 0;
      self_status.OTA_timeout = false;
      vTaskResume(task_OTA); // Resume the OTA task
    }

  } // end task loop
}

#pragma endregion
void setup()
{
#pragma region Setup Serial
  // Serial
  Serial_mutex = xSemaphoreCreateMutex();

  xSemaphoreTake(Serial_mutex, (TickType_t)10 / portTICK_PERIOD_MS);
  Serial.begin(115200);
  Serial.println("Startup...");
  xSemaphoreGive(Serial_mutex);
#pragma endregion
#pragma region Data Setup
  // FFStatus
  status_mutex = xSemaphoreCreateMutex();
#pragma endregion
#pragma region Setup OTA
  // OTA Task
  xTaskCreatePinnedToCore( // Use xTaskCreate() in vanilla FreeRTOS
      handleOTA,           // Function to be called
      "Handle OTA",        // Name of task
      2048,                // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                // Parameter to pass to function
      1,                   // Task priority (0 to configMAX_PRIORITIES - 1)
      &task_OTA,           // Task handle
      app_cpu);            // Run on one core for demo purposes (ESP32 only)

  Serial.println("OTA Task Started");
#pragma endregion
#pragma region Setup LED / Heartbeat
  // Configure LED
  pinMode(led_pin, OUTPUT);
  // LED Task
  xTaskCreatePinnedToCore( // Use xTaskCreate() in vanilla FreeRTOS
      toggleLED,           // Function to be called
      "Toggle LED",        // Name of task
      1024,                // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                // Parameter to pass to function
      1,                   // Task priority (0 to configMAX_PRIORITIES - 1)
      &task_LED,           // Task handle
      app_cpu);            // Run on one core for demo purposes (ESP32 only)

  Serial.println("LED Task Started");
#pragma endregion
#pragma region GPS Setup
  // Setup GPS
  if (!GPS.begin(9600))
  {
    if (xSemaphoreTake(Serial_mutex, (TickType_t)10 / portTICK_PERIOD_MS) == pdTRUE)
    {
      Serial.println("GPS ...!!!... [[FAIL]]");
      xSemaphoreGive(Serial_mutex);
    }
    self_status.GPS_connected = false;
  }
  else
  {
    if (xSemaphoreTake(Serial_mutex, (TickType_t)10 / portTICK_PERIOD_MS) == pdTRUE)
    {
      Serial.println("GPS ...///... [OK]");
      xSemaphoreGive(Serial_mutex);
    }
    self_status.GPS_connected = true;
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 second
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 10 Seconds
    //GPS.sendCommand(PGCMD_ANTENNA); //Request antennae status as well?
  }

  // GPS Tasks
  GPS_mutex = xSemaphoreCreateMutex();
  GPS_new = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore( //  "Update GPS"
      updateGPS,           // Function to be called
      "Update GPS",        // Name of task
      2048,                // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                // Parameter to pass to function
      1,                   // Task priority (0 to configMAX_PRIORITIES - 1)
      NULL,                // Task handle
      app_cpu);            // Run on one core for demo purposes (ESP32 only)

  Serial.println("GPS Task Started");
  // xTaskCreatePinnedToCore(  // "Print GPS"
  //   printGPS,    // Function to be called
  //   "Print GPS", // Name of task
  //   1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
  //   NULL,         // Parameter to pass to function
  //   1,            // Task priority (0 to configMAX_PRIORITIES - 1)
  //   NULL,         // Task handle
  //   app_cpu);     // Run on one core for demo purposes (ESP32 only)

  Serial.println("GPS Print Task Started");
#pragma endregion
#pragma region IMU Setup
  // IMU Tasks
  IMU085_mutex = xSemaphoreCreateMutex();
  IMU085_new = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore( // "Handle IMU085"
      handleIMU085,        // Function to be called
      "Handle IMU085",     // Name of task
      2048,                // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                // Parameter to pass to function
      1,                   // Task priority (0 to configMAX_PRIORITIES - 1)
      NULL,                // Task handle
      app_cpu);            // Run on one core for demo purposes (ESP32 only)

  Serial.println("IMU085 Task Started");
#pragma endregion
#pragma region TFT Setup
  // TFT Task
  TFT_mutex = xSemaphoreCreateMutex();
  TFT_new = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore( // "Handle TFT"
      handleTFT,           // Function to be called
      "Handle TFT",        // Name of task
      10000,               // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                // Parameter to pass to function
      1,                   // Task priority (0 to configMAX_PRIORITIES - 1)
      &task_TFT,           // Task handle
      app_cpu);            // Run on one core for demo purposes (ESP32 only)

  Serial.println("TFT Task Started");
#pragma endregion
#pragma region Knob / Buttons Setup
  xTaskCreatePinnedToCore( // "Handle TFT"
      handleKnob,          // Function to be called
      "Handle Knob",       // Name of task
      10000,               // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                // Parameter to pass to function
      1,                   // Task priority (0 to configMAX_PRIORITIES - 1)
      NULL,                // Task handle
      app_cpu);            // Run on one core for demo purposes (ESP32 only)

  Serial.println("TFT Task Started");
#pragma endregion
#pragma region Controller Setup
  xTaskCreatePinnedToCore( // "Handle TFT"
      handleControl,       // Function to be called
      "Controller",        // Name of task
      10000,               // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                // Parameter to pass to function
      1,                   // Task priority (0 to configMAX_PRIORITIES - 1)
      NULL,                // Task handle
      app_cpu);            // Run on one core for demo purposes (ESP32 only)

  Serial.println("Controller Task Started");
#pragma endregion

#pragma region Setup LORA
  // LORA Task
  xTaskCreatePinnedToCore( // Use xTaskCreate() in vanilla FreeRTOS
      handleLORA,          // Function to be called
      "Handle LORA",       // Name of task
      10000,               // Stack size (bytes in ESP32, words in FreeRTOS)
      NULL,                // Parameter to pass to function
      2,                   // Task priority (0 to configMAX_PRIORITIES - 1)
      &task_LORA,          // Task handle
      app_cpu);            // Run on one core for demo purposes (ESP32 only)

  Serial.println("LORA Task Started");
#pragma endregion

  Serial.println("All tasks created");
}

void loop()
{
  // Do nothing but allow yielding to lower-priority tasks
  // vTaskDelay(1000 / portTICK_PERIOD_MS);

  // setup() and loop() run in their own task with priority 1 in core 1
  // on ESP32
}
