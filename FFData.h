// Data Packet



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

// Haversine Distance Calculator
// from: http://forum.arduino.cc/index.php?topic=45760.msg332014#msg332014
int HaverSineFixed(uint32_t lat1, uint32_t lon1, uint32_t lat2, uint32_t lon2) {
 float flat1 = lat1/10000000;
 float flat2 = lat2/10000000;
 float flon1 = lon1/10000000;
 float flon2 = lon2/10000000;


 float ToRad = PI / 180.0;
 float R = 63710;   // radius earth in m

 float dLat = (lat2-lat1) * ToRad;
 float dLon = (lon2-lon1) * ToRad;

 float a = sin(dLat/2) * sin(dLat/2) +
       cos(lat1 * ToRad) * cos(lat2 * ToRad) *
       sin(dLon/2) * sin(dLon/2);

 float c = 2 * atan2(sqrt(a), sqrt(1-a));

 float d = ((R * c) * 8)/1000; // convert to meters
 int distance = d;
 return distance;
}

#define R 6371
#define TO_RAD (3.1415926536 / 180)
double dist(double th1, double ph1, double th2, double ph2) {
	double dx, dy, dz;
	ph1 -= ph2;
	ph1 *= TO_RAD, th1 *= TO_RAD, th2 *= TO_RAD;

	dz = sin(th1) - sin(th2);
	dx = cos(ph1) * cos(th1) - cos(th2);
	dy = sin(ph1) * cos(th1);
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * R;
}

int dumbDistance(uint32_t lat1, uint32_t lon1, uint32_t lat2, uint32_t lon2) {
  uint32_t dLonSq = (lon2 - lon1)^2;
  uint32_t dLatSq = (lat2 - lat1)^2;
  uint32_t dSum = dLonSq + dLatSq;
  long d = sqrt(dSum);
  d = (d*0.08);
  return d;
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
