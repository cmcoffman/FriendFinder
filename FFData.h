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
