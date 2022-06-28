# scratch
 void printEvent(sensors_event_t* event) {
  Serial.println();
  Serial.print(event->type);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  Serial.print(": x= ");
  Serial.print(x);
  Serial.print(" | y= ");
  Serial.print(y);
  Serial.print(" | z= ");
  Serial.println(z);
}


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


2000 / portTICK_PERIOD_MS
vTaskDelay(2000 / portTICK_PERIOD_MS);
xSemaphoreTake(Serial_mutex, portMAX_DELAY);
xSemaphoreTake(Serial_mutex, portMAX_DELAY);
if (xSemaphoreTake( Serial_mutex, ( TickType_t ) 10 / portTICK_PERIOD_MS ) == pdTRUE) {

if (eTaskGetState(task2_tid) != eSuspended) {


// IMU_085 
void printActivity(uint8_t activity_id)
{
  switch (activity_id)
  {
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

// void printGPS(void *parameter) {
//   while (1) {
//     xSemaphoreTake(GPS_new, portMAX_DELAY);
//     if (xSemaphoreTake(GPS_mutex, ( TickType_t ) 10 / portTICK_PERIOD_MS ) == pdTRUE) {
//       // Time in seconds keeps increasing after we get the NMEA sentence.
//       // This estimate will lag real time due to transmission and parsing delays,
//       // but the lag should be small and should also be consistent.
//       float s = GPS.seconds + GPS.milliseconds / 1000. + GPS.secondsSinceTime();
//       int m = GPS.minute;
//       int h = GPS.hour;
//       int d = GPS.day;
//       // Adjust time and day forward to account for elapsed time.
//       // This will break at month boundaries!!! Humans will have to cope with
//       // April 31,32 etc.
//       while (s > 60) {
//         s -= 60;
//         m++;
//       }
//       while (m > 60) {
//         m -= 60;
//         h++;
//       }
//       while (h > 24) {
//         h -= 24;
//         d++;
//       }
//     // ISO Standard Date Format, with leading zeros https://xkcd.com/1179/
//     Serial.print("\nDate: ");
//     Serial.print(GPS.year + 2000, DEC);
//     Serial.print("-");
//     if (GPS.month < 10)
//       Serial.print("0");
//     Serial.print(GPS.month, DEC);
//     Serial.print("-");
//     if (d < 10)
//       Serial.print("0");
//     Serial.print(d, DEC);
//     Serial.print("   Time: ");
//     if (h < 10)
//       Serial.print("0");
//     Serial.print(h, DEC);
//     Serial.print(':');
//     if (m < 10)
//       Serial.print("0");
//     Serial.print(m, DEC);
//     Serial.print(':');
//     if (s < 10)
//       Serial.print("0");
//     Serial.println(s, 3);
//     Serial.print("Fix: ");
//     Serial.print((int)GPS.fix);
//     Serial.print(" quality: ");
//     Serial.println((int)GPS.fixquality);
//     Serial.print("Time [s] since last fix: ");
//     Serial.println(GPS.secondsSinceFix(), 3);
//     Serial.print("    since last GPS time: ");
//     Serial.println(GPS.secondsSinceTime(), 3);
//     Serial.print("    since last GPS date: ");
//     Serial.println(GPS.secondsSinceDate(), 3);
//     if (GPS.fix) {
//       Serial.print("Location: ");
//       Serial.print(GPS.latitude, 4);
//       Serial.print(GPS.lat);
//       Serial.print(", ");
//       Serial.print(GPS.longitude, 4);
//       Serial.println(GPS.lon);
//       Serial.print("Speed (knots): ");
//       Serial.println(GPS.speed);
//       Serial.print("Angle: ");
//       Serial.println(GPS.angle);
//       Serial.print("Altitude: ");
//       Serial.println(GPS.altitude);
//       Serial.print("Satellites: ");
//       Serial.println((int)GPS.satellites);
//     }
//     xSemaphoreGive(GPS_mutex);
//     xSemaphoreGive(Serial_mutex);
//     vTaskDelay(2000 / portTICK_PERIOD_MS);
//   }
// }

// FFDisplay Radio Setup ##########################

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

// Radio
#define MY_ADDRESS 2
#define THEIR_ADDRESS 1
#define RFM95_CS 21
#define RFM95_RST 2
#define RFM95_INT 32
RH_RF95 driver(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(driver, MY_ADDRESS);


void setup() {
      // Reset Radio
  digitalWrite(RFM95_RST, HIGH);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  delay(1000);

  // Setup HSPI
#define HSPI_SCK 25
#define HSPI_MISO 27
#define HSPI_MOSI 26
  SPI.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI);
  delay(1000);


  if (!manager.init()) {
    Serial.println("Radio - FAIL");
  } else {
    Serial.println("Radio - OK");
  }

  if (!driver.setFrequency(915.0)) {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }

  // You can set transmitter powers from 5 to 23 dBm:
  driver.setTxPower(23, false);
  driver.setCADTimeout(5000);

}

uint8_t data[] = "I exist!";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

