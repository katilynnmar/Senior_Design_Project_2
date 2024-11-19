#include <TinyGPS++.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

// GPS and IMU objects
TinyGPSPlus gps;
Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;

bool logGPS = true; // Flag to alternate between GPS and IMU

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600); // GPS Serial
  Serial.println("GPS and IMU Start");

  // Initialize IMU sensors
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("Failed to initialize LSM6DSOX!");
    while (1);
  }
  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to initialize LIS3MDL!");
    while (1);
  }

  // Print CSV headers
  Serial.println("Data_Type,Satellite_Count,Latitude,Longitude,Speed_MPH,Altitude_Feet,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Mag_X,Mag_Y,Mag_Z,Temperature");
}

void loop()
{
  if (logGPS) {
    // Process GPS data
    bool gpsLogged = false; // Flag to track if GPS data was logged
    while (Serial1.available()) {
      gps.encode(Serial1.read());
      if (gps.location.isUpdated()) {
        // Print GPS data in CSV format
        Serial.print("GPS,");
        Serial.print(gps.satellites.value()); Serial.print(",");
        Serial.print(gps.location.lat(), 6); Serial.print(",");
        Serial.print(gps.location.lng(), 6); Serial.print(",");
        Serial.print(gps.speed.mph()); Serial.print(",");
        Serial.print(gps.altitude.feet()); Serial.print(",");
        // Fill IMU fields with empty values
        Serial.print(",,,,,,,,,,");
        Serial.println();
        gpsLogged = true; // GPS data successfully logged
        break;
      }
    }
    if (gpsLogged) {
      logGPS = false; // Switch to IMU for the next cycle
    }
  } else {
    // Process IMU data
    sensors_event_t accel, gyro, mag, temp;
    lsm6dsox.getEvent(&accel, &gyro, &temp);
    lis3mdl.getEvent(&mag);

    // Print IMU data in CSV format
    Serial.print("IMU,,,,,,"); // Leave GPS fields empty
    Serial.print(accel.acceleration.x, 4); Serial.print(",");
    Serial.print(accel.acceleration.y, 4); Serial.print(",");
    Serial.print(accel.acceleration.z, 4); Serial.print(",");
    Serial.print(gyro.gyro.x, 4); Serial.print(",");
    Serial.print(gyro.gyro.y, 4); Serial.print(",");
    Serial.print(gyro.gyro.z, 4); Serial.print(",");
    Serial.print(mag.magnetic.x, 4); Serial.print(",");
    Serial.print(mag.magnetic.y, 4); Serial.print(",");
    Serial.print(mag.magnetic.z, 4); Serial.print(",");
    Serial.println(temp.temperature);
    logGPS = true; // Switch back to GPS for the next cycle
  }

  delay(500); // Adjust the delay as needed
}
