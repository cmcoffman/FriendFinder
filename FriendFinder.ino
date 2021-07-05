#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPI.h>
#include <TFT_eSPI.h>


// FreeRTOS
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Serial Terminal Output
static SemaphoreHandle_t Serial_mutex;   // Locks Serial object


// GPS
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false // Echo GPS to Serial terminal?
bool GPS_connected = false;
static SemaphoreHandle_t GPS_mutex;   // Locks GPS object
static SemaphoreHandle_t GPS_new;     //Signals NEW GPS data

void updateGPS(void *parameter) {
  while(1) {
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
      xSemaphoreGive(GPS_new);
    } 
  }
   xSemaphoreGive(GPS_mutex);
  }
}

void printGPS(void *parameter) {
  while(1) {
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


// LED
static const int led_pin = LED_BUILTIN;
void toggleLED(void *parameter) {
  while(1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


// IMU
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x28); // id, address
sensors_event_t orientationData , linearAccelData;
double xPos = 0, yPos = 0, headingVel = 0;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500; // how often to print the data
//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
bool IMU_connected = false;
static SemaphoreHandle_t IMU_mutex;   // Locks IMU object
static SemaphoreHandle_t IMU_new;     //Signals NEW IMU data

void updateIMU(void *parameter) {
  uint16_t printCount = 0; //counter to avoid printing every 10MS sample
  while(1) {
    unsigned long tStart = micros();
    
    xSemaphoreTake(IMU_mutex, portMAX_DELAY);
    IMU.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    //  bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    IMU.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    xPos = xPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
    yPos = yPos + ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;

    // velocity of sensor in the direction it's facing
    headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);

    // Data Collection Complete
    xSemaphoreGive(IMU_mutex);
    xSemaphoreGive(IMU_new);

    // Print Results (should be it's own task?)
    if (printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
      //enough iterations have passed that we can print the latest data
      xSemaphoreTake(Serial_mutex, portMAX_DELAY);
      Serial.print("Heading: ");
      Serial.println(orientationData.orientation.x);
      Serial.print("Position: ");
      Serial.print(xPos);
      Serial.print(" , ");
      Serial.println(yPos);
      Serial.print("Speed: ");
      Serial.println(headingVel);
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
  while(1) {
    // Line 1 (Time)
    MSD_screen.setCursor(0, 0);
    MSD_screen.setTextColor(TFT_DARKGREY, TFT_BLACK);
    MSD_screen.print(F("TIME // [UNDEFINED]"));
    
    // Line 2 (Space)
    MSD_screen.setCursor(0, 16);
    if(!GPS_connected) MSD_screen.setTextColor(TFT_DARKGREY, TFT_BLACK);
    if(GPS_connected) MSD_screen.setTextColor(TFT_ORANGE, TFT_BLACK);
    xSemaphoreTake(GPS_new, portMAX_DELAY);
    xSemaphoreTake(GPS_mutex, portMAX_DELAY);
    xSemaphoreTake(TFT_mutex, portMAX_DELAY);
    if(!GPS.fix) MSD_screen.print(F("SPACE // [INVALID]"));
    if(GPS.fix) {
      MSD_screen.print(F("SPACE // "));
      MSD_screen.print(GPS.latitude, 7);
      MSD_screen.print(F(", "));
      MSD_screen.print(GPS.longitude, 7);
    }
    xSemaphoreGive(GPS_mutex);
    MSD_screen.pushSprite(0, 0); 
    xSemaphoreGive(TFT_mutex);

    vTaskDelay(16 / portTICK_PERIOD_MS); // 16 ms should be ~60 fps
  }
}

void setup() {
  // Serial
  Serial_mutex = xSemaphoreCreateMutex();
  xSemaphoreTake(Serial_mutex, portMAX_DELAY);
  Serial.begin(115200);
  Serial.println("Startup...");
  xSemaphoreGive(Serial_mutex);

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
  if (!IMU.begin()) {
    IMU_connected = false;
    xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    Serial.println("IMU ...!!!... [[FAIL]]");
    xSemaphoreGive(Serial_mutex);  
  } else {
    IMU_connected = true;
    xSemaphoreTake(Serial_mutex, portMAX_DELAY);
    Serial.println("IMU ...!!!... [[FAIL]]");
    xSemaphoreGive(Serial_mutex);
  }
  
  // IMU Tasks
  IMU_mutex = xSemaphoreCreateMutex();
  IMU_new = xSemaphoreCreateBinary();
  //xSemaphoreTake(IMU_new, portMAX_DELAY);

  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
                updateIMU,    // Function to be called
                "Update IMU", // Name of task
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

    