#include <TinyGPS++.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <Adafruit_LSM6DSOX.h>   // Library for LSM6DSOX IMU
#include <Adafruit_LIS3MDL.h>    // Library for LIS3MDL magnetometer

// Define pin connections
#define HC06_RX 34              // Teensy Pin RX2 for HC-06 Bluetooth module
#define HC06_TX 35              // Teensy Pin TX2 for HC-06 Bluetooth module
#define SD_CS_PIN BUILTIN_SDCARD // Pin for the built-in SD card reader on Teensy

// Declare sensor and communication objects
Adafruit_LSM6DSOX lsm6dsox;       // IMU object for LSM6DSOX
Adafruit_LIS3MDL lis3mdl;         // Magnetometer object for LIS3MDL
TinyGPSPlus gps;                  // GPS object to handle NMEA data
File dataFile;                    // File object for SD card logging
SoftwareSerial hc06(HC06_RX, HC06_TX); // SoftwareSerial for HC-06 Bluetooth module

void setup()
{
  Serial.begin(9600);          // Initialize Serial Monitor for debugging
  Serial7.begin(9600);         // Initialize GPS Serial (GPS TX -> Teensy RX1)
  hc06.begin(115200);          // Initialize HC-06 Bluetooth Serial
  Serial.println("GPS Start"); // Indicate the system has started

  // Display system initialization message
  Serial.println("System initializing...");
  hc06.println("Bluetooth communication active.");

  // Wait for Serial Monitor connection
  while (!Serial) delay(10);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Failed to initialize SD card!");
    hc06.println("Failed to initialize SD card!");
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
  dataFile = SD.open("gpslog.csv", FILE_WRITE);
  if (dataFile) {
    // Write column headers to the CSV file
    dataFile.print("Satellite_Count,Latitude,Longitude,Speed_MPH,Altitude_Feet,");
    dataFile.println("Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Mag_X,Mag_Y,Mag_Z,Temperature");
    dataFile.flush(); // Ensure headers are written immediately
    Serial.println("SD card initialized. GPS and IMU data logging started.");
    hc06.println("SD card initialized. GPS and IMU data logging started.");
  } else {
    Serial.println("Error opening gpslog.csv!");
    hc06.println("Error opening gpslog.csv!");
    while (1); // Halt execution if file opening fails
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

      // Write GPS and IMU data to the SD card
      if (dataFile) {
        dataFile.println(gpsData); // Log GPS data
        dataFile.println(imuData); // Log IMU data
        dataFile.flush();          // Ensure data is written to the file
      } else {
        Serial.println("Error writing to gpslog.csv!"); // SD write error
        hc06.println("Error writing to gpslog.csv!");
      }

      // Print GPS and IMU data to the Serial Monitor
      Serial.print("GPS, ");
      Serial.println(gpsData);
      Serial.print("IMU, ");
      Serial.println(imuData);

      // Send GPS and IMU data to Bluetooth
      hc06.println(gpsData);
      hc06.println(imuData);

      // Handle case when no more GPS data is available
      if (!Serial7.available()) {
        Serial.println("No GPS data available. Waiting for signal...");
        hc06.println("No GPS data available. Waiting for signal...");
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
