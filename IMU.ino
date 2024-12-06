#include <Adafruit_LSM6DSOX.h>    // Library for IMU
#include <Adafruit_LIS3MDL.h>     // Library for magnetometer
#include <SD.h>                   // Library for SD card
#include <SoftwareSerial.h>


#define SD_CS_PIN BUILTIN_SDCARD // Pin for the built-in SD card reader on Teensy

// Objects for IMU and SD card logging
Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;
File dataFile;


void setup() {
  // Initialize Serial Monitors and Bluetooth
  Serial.begin(9600);
  Serial1.begin(9600);      // HC-06 default baud rate is 9600

  
  Serial.println("System initializing...");
  Serial1.println("Bluetooth communication active.");
  
  // Wait for Serial Monitor
  while (!Serial) delay(10);

  // Initialize IMU sensors
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("Failed to initialize LSM6DSOX!");
    while (1);
  }
  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to initialize LIS3MDL!");
    while (1);
  }
  Serial.println("IMU sensors initialized successfully.");

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Failed to initialize SD card!");
    while (1); // Stop if the SD card doesn't initialize
  }
  
  dataFile = SD.open("imulog.csv", FILE_WRITE); // Open the CSV file
  if (dataFile) {
    dataFile.println("Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Mag_X,Mag_Y,Mag_Z,Temperature");
    dataFile.flush();
    Serial.println("SD card initialized. IMU data logging started.");
  } else {
    Serial.println("Error opening imulog.csv!");
    while (1);
  }
}

void loop() {
  // Process IMU data
  sensors_event_t accel, gyro, mag, temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  // Format IMU data
  String imuData = String(accel.acceleration.x, 4) + "," +
                   String(accel.acceleration.y, 4) + "," +
                   String(accel.acceleration.z, 4) + "," +
                   String(gyro.gyro.x, 4) + "," +
                   String(gyro.gyro.y, 4) + "," +
                   String(gyro.gyro.z, 4) + "," +
                   String(mag.magnetic.x, 4) + "," +
                   String(mag.magnetic.y, 4) + "," +
                   String(mag.magnetic.z, 4) + "," +
                   String(temp.temperature, 2);

  // Log IMU data to SD card, Serial Monitor, and Bluetooth
  if (dataFile) {
    dataFile.println(imuData);
    dataFile.flush(); // Ensure immediate write to the SD card
  } else {
    Serial.println("Error writing to imulog.csv!");
  }
  Serial.println(imuData);
  Serial1.println(imuData);

  delay(1000); // Adjust delay as needed
}

void closeFile() {
  if (dataFile) {
    dataFile.close(); // Close file to ensure data integrity
  }
}
