
#include "Arduino.h"
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_NeoPixel.h>

// Serial Stuff
void initSerial() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {
    delay(1);
  }
  Serial.println("Booting up...");
  Serial.println("Serial Port [OK]");
  delay(100);
}

// Neopixel Stuff
#define LEDPIN 5  // Pin Neopixel Ring is on
#define NUMPIXELS 24 // Number of Pixels
#define BRIGHTNESS 1
#define TOPPIXEL 15

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

uint32_t RED = strip.Color(255, 0, 0);
uint32_t GREEN = strip.Color(0, 255, 0);
uint32_t BLUE = strip.Color(0, 0, 255);
uint32_t YELLOW = strip.Color(255, 255, 0);
uint32_t WHITE = strip.Color(255, 255, 255);
uint32_t GREY = strip.Color(64, 64, 64);
uint32_t OFF = strip.Color(0, 0, 0);
uint32_t YELLOWGREEN = strip.Color(128, 255, 0);
uint32_t PURPLE = strip.Color(255, 0, 255);

void colorDot(int pixel, uint32_t color) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    if (i == pixel) {
      strip.setPixelColor(i, color);
    } else {
      strip.setPixelColor(i, OFF);
    }
    strip.show();
  }
}

void colorDotBuff(int pixel, uint32_t color) {

      strip.setPixelColor(pixel, color);
}

void colorFill(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}

void colorFillBuff(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
}

void initNeopixels() {
  Serial.print("Neopixels");
  colorFill(OFF);
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show();
  Serial.println("...[OK]");
}

void Blink(byte PIN, byte wait, byte loops) {
  for (byte i = 0; i < loops; i++)  {
    digitalWrite(PIN, HIGH);
    delay(wait);
    digitalWrite(PIN, LOW);
    delay(wait);
  }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

void colorDotWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.setPixelColor(i - 1, OFF);
    strip.show();
    delay(wait);
  }
  strip.setPixelColor(strip.numPixels() - 1, OFF);
  strip.show();
}

void colorMeter(uint32_t c, int amount) {
  amount = map(amount, 0, 100, 23, 0);
  for (int i = 0; i < 24; i++) {
    if (i <= amount) {
    strip.setPixelColor(i, c);
    } else {
    strip.setPixelColor(i, OFF);
    }  
  }
  
  strip.show();
  //Serial.print("Amount: ");
  //Serial.println(amount);

}

void colorMeterBuff(uint32_t c, int amount) {
  amount = map(amount, 0, 100, 0, 23);
  for (int i = 0; i < 24; i++) {
    if (i <= amount) {
    strip.setPixelColor(i, c);
    } else {
    strip.setPixelColor(i, OFF);
    }  
  }
  
  //Serial.print("Amount: ");
  //Serial.println(amount);

}

// IMU Stuff
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Sample Rate
#define BNO055_SAMPLERATE_DELAY_MS (100)

float heading;

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


// Radio Stuff
// for Prototype Board
#define RFM95_CS 6
#define RFM95_RST 11
#define RFM95_INT 10


/* for Feather32u4 RFM9x
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 7
*/

/* for feather m0 RFM9x
  #define RFM95_CS 8
  #define RFM95_RST 4
  #define RFM95_INT 3
*/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13


// Data Packet
struct dataPacket {
  float latitudeDegrees;      // 4 bytes
  float longitudeDegrees;     // 4 bytes
  uint8_t fixquality;                // 1 byte
  uint8_t satellites;                // 1 byte
  uint8_t hour;               // 1 byte
  uint8_t minute;                // 1 byte
  uint8_t seconds;                // 1 byte
};

struct dataPacket newDataPacket;

void initRadio() {
  Serial.print("Radio...");
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("[FAIL]");
    while (1);
  }


  //  while (!rf95.init()) {
  //    Serial.print(".");
  //  }
  //  while (!rf95.available()) {
  //    Serial.print(".");
  //  }

  Serial.println("[OK]");

  Serial.print("Setting radio frequency...");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("[FAIL]");
    while (1);
  }
  Serial.println("[OK]");

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

bool checkRadio(bool verbose = true) {
  //Serial.println("Radio: check");
  if (rf95.available()) {
    if (verbose) Serial.println("Radio: available");
    // Should be a message for us now
    //uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(newDataPacket);

    if (rf95.recv((uint8_t *)&newDataPacket, &len)) {
      if (verbose) Serial.println("Radio: Message received!");
      //colorFill(WHITE);
      digitalWrite(LED, HIGH);
      delay(10);
      digitalWrite(LED, LOW);
      //colorFill(OFF);
      if (verbose) {
        RH_RF95::printBuffer("Received: ", buf, len);
        //Serial.print("Got: ");
        //Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);

        if (newDataPacket.fixquality != 0) { 
        Serial.print("GPS Fix: ");
        Serial.print(newDataPacket.latitudeDegrees, 8);
        Serial.print(", ");
        Serial.println(newDataPacket.longitudeDegrees, 8);
        } else {
          Serial.println("Friend has no GPS fix.");
        }
      }
    return true;
    }
    else
    {
      if (verbose) Serial.println("Radio: No message received.");
      return false;
    }
  }
}

// GPS Stuff
// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false



void initGPS() {
  Serial.print("GPS...");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  // PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F" // Once every 10 seconds, 100 millihertz.
  // PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B"  // Once every 5 seconds, 200 millihertz.
  // PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
  // PMTK_SET_NMEA_UPDATE_2HZ  "$PMTK220,500*2B"
  // PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
  // PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ);

  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  //useInterrupt(true);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  Serial.println("[OK]");
}

void checkGPS(bool verbose = true) {
  GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    if (verbose) Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      if (verbose) Serial.println("GPS - Parse failure.");
    return;  // we can fail to parse a sentence in which case we should just wait for another
  }
}

void printGPS() {
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    } else {
      Serial.println("-No GPS Fix.-");
    }
}

// Haversine Distance Calculator
// from: http://forum.arduino.cc/index.php?topic=45760.msg332014#msg332014
int HaverSine(float lat1, float lon1, float lat2, float lon2) {
 float ToRad = PI / 180.0;
 float R = 6371;   // radius earth in Km
 
 float dLat = (lat2-lat1) * ToRad;
 float dLon = (lon2-lon1) * ToRad; 
 
 float a = sin(dLat/2) * sin(dLat/2) +
       cos(lat1 * ToRad) * cos(lat2 * ToRad) * 
       sin(dLon/2) * sin(dLon/2); 
       
 float c = 2 * atan2(sqrt(a), sqrt(1-a)); 
 
 float d = (R * c) * 1000; // convert to meters
 int distance = d;
 return distance;
}

// Compass Stuff ##########################
sensors_event_t event;

void checkIMU() {
  bno.getEvent(&event);
}

int orientRing (int heading) {
  // Get degrees per pixel
  int degPerPixel = 360 / strip.numPixels();
  // Calculate pixel for heading
  int pixelOut = -((heading / degPerPixel) - TOPPIXEL);
  pixelOut = (pixelOut + 24) % 24;
  return (pixelOut);
}

int findNorth() {
  return event.orientation.x;
}


void drawCompass (bool verbose = false) {
  int heading = event.orientation.x;
  int pixel = orientRing(heading);
  // Get degrees per pixel
  int degPerPixel = 360 / strip.numPixels();
  // Calculate pixel for heading
  int pixelOut = -((heading / degPerPixel) - TOPPIXEL);
  pixelOut = (pixelOut + 24) % 24;
  colorDotBuff(pixelOut, YELLOW);
  if (verbose) {
    Serial.print("North Pixel: ");
    Serial.println(pixelOut);
  }
}

void drawDistance(bool verbose = false) {
    int distance;
    distance = HaverSine(GPS.latitudeDegrees, GPS.longitudeDegrees, newDataPacket.latitudeDegrees, newDataPacket.longitudeDegrees);
    if (verbose) {
      Serial.print("Distance: ");
      Serial.println(distance); 
    }
    if(distance <= 100) colorMeter(GREY, distance);
    //strip.show();
}

int bearing(float lat,float lon,float lat2,float lon2) {
  // Gotten from: https://gis.stackexchange.com/questions/252672/calculate-bearing-between-two-decimal-gps-coordinates-arduino-c?newreg=eb676d9dca8f4cc8ad10c14a3b00d423
    
    
    float teta1 = radians(lat);
    float teta2 = radians(lat2);
    float delta1 = radians(lat2-lat);
    float delta2 = radians(lon2-lon);

    //==================Heading Formula Calculation================//

    float y = sin(delta2) * cos(teta2);
    float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    int brng = atan2(y,x);
    brng = degrees(brng);  // radians to degrees
    brng = ( ((int)brng + 360) % 360 ); 

    // Serial.print("Heading GPS: ");
    // Serial.println(brng);

    return brng;
  }


void drawFriend(bool verbose = false) {
  int friendHeading = bearing(GPS.latitudeDegrees, GPS.longitudeDegrees, newDataPacket.latitudeDegrees, newDataPacket.longitudeDegrees);
  int northHeading = findNorth();
  int pixelOut;
  if (friendHeading != northHeading) {

  int heading = northHeading - friendHeading;
  int pixel = orientRing(heading);
  // Get degrees per pixel
  int degPerPixel = 360 / strip.numPixels();
  // Calculate pixel for heading
  pixelOut = -((heading / degPerPixel) - TOPPIXEL);
  pixelOut = (pixelOut + 24) % 24;
  colorDotBuff(pixelOut, GREEN);
  } else {
    int heading = northHeading + friendHeading;
  int pixel = orientRing(heading);
  // Get degrees per pixel
  int degPerPixel = 360 / strip.numPixels();
  // Calculate pixel for heading
  pixelOut = -((heading / degPerPixel) - TOPPIXEL);
  pixelOut = (pixelOut + 24) % 24;
  colorDotBuff(pixelOut, YELLOWGREEN);
  }
  if (verbose) {
    Serial.print("Friend Pixel: ");
    Serial.println(pixelOut);
  }
  
}

void printFriend() {
  int heading = bearing(GPS.latitude, GPS.longitude, newDataPacket.latitudeDegrees, newDataPacket.longitudeDegrees);
  int distance = HaverSine(GPS.latitudeDegrees, GPS.longitudeDegrees, newDataPacket.latitudeDegrees, newDataPacket.longitudeDegrees);
  
  Serial.print("My GPS: ") ;
  Serial.print(GPS.latitudeDegrees, 6);
  Serial.print(", ");
  Serial.println(GPS.longitudeDegrees, 6);


  Serial.print("Friend GPS: ") ;
  Serial.print(newDataPacket.latitudeDegrees, 6);
  Serial.print(", ");
  Serial.println(newDataPacket.longitudeDegrees, 6);

  Serial.print("Friend Distance (m): ");
  Serial.println(distance);
  
  Serial.print("Friend Bearing (deg): ");
  Serial.println(heading);
  
}