#include <Adafruit_LSM6DSOX.h>   // Library for LSM6DSOX IMU
#include <Adafruit_LIS3MDL.h>     // Library for LIS3MDL magnetometer
//uses I2CS
Adafruit_LSM6DSOX lsm6ds;         // IMU object for accelerometer and gyroscope
Adafruit_LIS3MDL lis3mdl;         // Magnetometer object

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);      // Wait until serial connection is ready

  Serial.println("Initializing IMU sensors...");

  // Initialize the LSM6DS for accelerometer and gyroscope
  if (!lsm6ds.begin_I2C()) {
    Serial.println("Failed to initialize LSM6DS sensor!");
    while (1) delay(10);
  }

  // Initialize the LIS3MDL for magnetometer
  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to initialize LIS3MDL sensor!");
    while (1) delay(10);
  }

  Serial.println("LSM6DS and LIS3MDL initialized successfully!");
}

void loop() {
  // Define structures to hold sensor events
  sensors_event_t accel, gyro, mag, temp;

  // Capture data from each sensor
  lsm6ds.getEvent(&accel, &gyro, &temp);  // Get accelerometer, gyroscope, and temperature data
  lis3mdl.getEvent(&mag);                 // Get magnetometer data

  // Display Acceleration Data (m/s^2)
  Serial.print("Acceleration (m/s^2) -> X: ");
  Serial.print(accel.acceleration.x, 4);
  Serial.print(", Y: ");
  Serial.print(accel.acceleration.y, 4);
  Serial.print(", Z: ");
  Serial.println(accel.acceleration.z, 4);

  // Display Angular Velocity (Gyroscope data in rad/s)
  Serial.print("Angular Velocity (rad/s) -> X: ");
  Serial.print(gyro.gyro.x, 4);
  Serial.print(", Y: ");
  Serial.print(gyro.gyro.y, 4);
  Serial.print(", Z: ");
  Serial.println(gyro.gyro.z, 4);

  // Display Magnetic Field Data (μT)
  Serial.print("Magnetic Field (μT) -> X: ");
  Serial.print(mag.magnetic.x, 4);
  Serial.print(", Y: ");
  Serial.print(mag.magnetic.y, 4);
  Serial.print(", Z: ");
  Serial.println(mag.magnetic.z, 4);

  // Optional: Display Temperature (in degrees Celsius)
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.println();  // New line for readability
  delay(1000);       // Delay to control data output rate
}
