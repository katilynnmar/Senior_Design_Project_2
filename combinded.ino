#include <TinyGPS++.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_LSM6DSOX.h>   // Library for LSM6DSOX IMU
#include <Adafruit_LIS3MDL.h>    // Library for LIS3MDL magnetometer

#define SD_CS_PIN BUILTIN_SDCARD // Pin for the built-in SD card reader on Teensy

// Declare sensor and communication objects
Adafruit_LSM6DSOX lsm6dsox;       // IMU object for LSM6DSOX
Adafruit_LIS3MDL lis3mdl;         // Magnetometer object for LIS3MDL
TinyGPSPlus gps;                  // GPS object to handle NMEA data
File dataFile;                    // File object for SD card logging

void setup()
{
  Serial.begin(9600);          // Initialize Serial Monitor for debugging
  Serial7.begin(9600);         // Initialize GPS Serial (GPS TX -> Teensy RX1)
  Serial1.begin(115200);          // Initialize HC-06 Bluetooth Serial
  Serial.println("GPS Start"); // Indicate the system has started

  // Display system initialization message
  Serial.println("System initializing...");
  Serial1.println("Bluetooth communication active.");

  // Wait for Serial Monitor connection
  while (!Serial) delay(10);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Failed to initialize SD card!");
    Serial1.println("Failed to initialize SD card!");
    while (1); // Halt execution if SD card initialization fails
  }

  // Initialize IMU (LSM6DSOX) sensor
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("Failed to initialize LSM6DSOX!");
    while (1); // Halt execution if IMU initialization fails
  }

  // Initialize magnetometer (LIS3MDL) sensor
  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to initialize LIS3MDL!");
    while (1); // Halt execution if magnetometer initialization fails
  }
  Serial.println("IMU sensors initialized successfully.");

  // Open CSV file for data logging on the SD card
  dataFile = SD.open("combolog.csv", FILE_WRITE); // Open the CSV file
  if (dataFile) {
    // Write CSV header
    dataFile.println("Satellite_Count,Latitude,Longitude,Altitude_Feet,Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Mag_X,Mag_Y,Mag_Z,Temperature");
    dataFile.flush();
    Serial.println("SD card initialized. GPS and IMU data logging started.");
    Serial1.println("SD card initialized. GPS and IMU data logging started.");
  } else {
    Serial.println("Error opening gpsimu_log.csv!");
    Serial1.println("Error opening gpsimu_log.csv!");
    while (1);
  }
}

void loop()
{
  sensors_event_t accel, gyro, mag, temp; // Objects to hold sensor data
  lsm6dsox.getEvent(&accel, &gyro, &temp); // Retrieve IMU data
  lis3mdl.getEvent(&mag);                 // Retrieve magnetometer data

  // Check if GPS data is available
  while (Serial7.available()) {
    gps.encode(Serial7.read()); // Process GPS NMEA data one character at a time

    // If GPS location data is updated
    if (gps.location.isUpdated()) {
      // Format GPS data into a comma-separated string
      String gpsData = String(gps.satellites.value()) + "," +
                       String(gps.location.lat(), 6) + "," +
                       String(gps.location.lng(), 6) + "," +
                       String(gps.speed.mph()) + "," +
                       String(gps.altitude.feet());

      // Format IMU data into a comma-separated string
      String imuData = String(accel.acceleration.x, 4) + "," +
                       String(accel.acceleration.y, 4) + "," +
                       String(accel.acceleration.z, 4) + "," +
                       String(gyro.gyro.x, 4) + "," +
                       String(gyro.gyro.y, 4) + "," +
                       String(gyro.gyro.z, 4) + "," +
                       String(mag.magnetic.x, 4) + "," +
                       String(mag.magnetic.y, 4) + "," +
                       String(mag.magnetic.z, 4) + "," +
                       String(temp.temperature, 1);

      String comboData = gpsData + "/" + imuData;

      // Write GPS and IMU data to the SD card
      if (dataFile) {
        dataFile.println(comboData); // Log GPS and IMU data
        dataFile.flush();          // Ensure data is written to the file
      } else {
        Serial.println("Error writing to combolog.csv!"); // SD write error
        Serial1.println("Error writing to combolog.csv!");
      }


      Serial.println(comboData);


      // Send GPS and IMU data to Bluetooth
      Serial1.println(comboData);

      // Handle case when no more GPS data is available
      if (!Serial7.available()) {
        Serial.println("No GPS data available. Waiting for signal...");
        Serial1.println("No GPS data available. Waiting for signal...");
      }

      Serial.println(); // Add a blank line for readability
    }
  }
}

// Function to close the file on SD card to ensure data integrity
void closeFile() {
  if (dataFile) {
    dataFile.close(); // Close the file safely
  }
}
