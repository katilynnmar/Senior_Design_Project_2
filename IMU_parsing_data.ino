#include <Adafruit_LSM6DSOX.h>   // Library for LSM6DSOX IMU
#include <Adafruit_LIS3MDL.h>     // Library for LIS3MDL magnetometer

// Create objects for the IMU sensors
Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;

void setup() {
  Serial.begin(115200);
  delay(100);                   
  while (!Serial) delay(10);     // Wait for Serial Monitor to be ready

  Serial.println("Initializing IMU sensors...");

  // Initialize the LSM6DSOX (accelerometer and gyroscope)
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("Failed to initialize LSM6DSOX sensor!");
    while (1) delay(10);         // Stop if initialization fails
  } else {
    Serial.println("LSM6DSOX initialized successfully.");
  }

  // Initialize the LIS3MDL (magnetometer)
  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to initialize LIS3MDL sensor!");
    while (1) delay(10);         // Stop if initialization fails
  } else {
    Serial.println("LIS3MDL initialized successfully.");
  }

  Serial.println("LSM6DSOX and LIS3MDL initialized successfully!\n");
  
  // Print CSV header
  Serial.println("Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Mag_X,Mag_Y,Mag_Z,Temperature");
}

void loop() {
  sensors_event_t accel, gyro, mag, temp;

  // Retrieve sensor events for accelerometer, gyroscope, and temperature
  lsm6dsox.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);        // Get magnetometer data

  // Print data in CSV format
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

  delay(1000); // Delay for readability and data pacing
}
