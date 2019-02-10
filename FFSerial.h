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
