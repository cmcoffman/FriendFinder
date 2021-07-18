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