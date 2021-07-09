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