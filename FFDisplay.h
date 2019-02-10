





int orientRing (int heading) {
  // Get degrees per pixel
  int degPerPixel = 360 / strip.numPixels();
  // Calculate pixel for heading
  int pixelOut = -((heading / degPerPixel) - TOPPIXEL);
  pixelOut = (pixelOut + 24) % 24;
  return (pixelOut);
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
