#include <Wire.h>       // For I2C communication with the IMU
#include <SPI.h>        // For SPI communication with SD card
#include <SD.h>         // For SD card access
#include <Adafruit_LSM6DSOX.h> //IMU 
#include <Adafruit_LIS3MDL.h> //Magnetometer
//#include <.h>  Replace with your GPS's library

Adafruit_LSM6DSOX lsm6dsox;    //IMU object
Adafruit_LIS3MDL lis3mdl;   //Magetometer object
//SomeGPSLibrary gps;    GPS object

File dataFile;

void setup() {
  // Initialize Serial for USB communication
  Serial.begin(9600);
  while (!Serial); // Wait for Serial connection

  // Initialize IMU (I2C)
  Wire.begin();
  if (!lsm6dsox.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1); // Stop if the IMU doesn't initialize
  }
  else if(!lis3mdl.begin()){
    Serial.println("Failed ot initialize Magnetometer")
  }


  // Initialize GPS (Serial1)
  Serial1.begin(9600);  // Adjust baud rate as per your GPS module's requirements
  if (!gps.begin(Serial1)) {
    Serial.println("Failed to initialize GPS!");
    while (1); // Stop if the GPS doesn't initialize
  }

  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("Failed to initialize SD card!");
    while (1); // Stop if the SD card doesn't initialize
  }

  // Open or create a new file on the SD card
  dataFile = SD.open("datalog.csv", FILE_WRITE);
  if (dataFile) {
    // Write headers to the CSV file
    dataFile.println("AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,GPS_Data");
    dataFile.flush();
    Serial.println("SD card initialized. Logging data...");
  } else {
    Serial.println("Error opening datalog.csv!");
    while (1);
  }
}

void loop() {
  String dataString = "";

  // Check and read IMU data
  if (lsm6dsox.available()) {
    float accelX = lsm6dsox.readAccelX();
    float accelY = lsm6dsox.readAccelY();
    float accelZ = lsm6dsox.readAccelZ();
    float gyroX = lsm6dsox.readGyroX();
    float gyroY = lsm6dsox.readGyroY();
    float gyroZ = lsm6dsox.readGyroZ();
    
    // Stream IMU data to the computer
    Serial.print("IMU: ");
    Serial.print("AccelX: "); Serial.print(accelX);
    Serial.print(", AccelY: "); Serial.print(accelY);
    Serial.print(", AccelZ: "); Serial.print(accelZ);
    Serial.print(", GyroX: "); Serial.print(gyroX);
    Serial.print(", GyroY: "); Serial.print(gyroY);
    Serial.print(", GyroZ: "); Serial.println(gyroZ);

    // Append IMU data to CSV line
    dataString += String(accelX) + "," + String(accelY) + "," + String(accelZ) + ",";
    dataString += String(gyroX) + "," + String(gyroY) + "," + String(gyroZ) + ",";
  }

  if(lis3mdl.available()){
    float magX = lis3mdl.readx();
    float magY = lis3mdl.ready();
    float magZ = lis3mdl.readz();

    //Stream Magentometer data to the computer
    Serial.print("Magnetometer: ");
    Serial.print("Magnetic X: "); Serial.print(magX);
    Serial.print(", Magnetic Y: "); Serial.print(magY);
    Serial.print(", Magnetic Z: "); Serial.print(magZ);

    //Append Magentometer data to CSV
    dataString += String(magX) + "," + String(magY) + "," + String(magZ) 
    
  }

  // Check and read GPS data
  if (gps.available()) {
    String gpsData = gps.readString();  // Assuming GPS library provides data as a string
    
    // Stream GPS data to the computer
    Serial.print("GPS: ");
    Serial.println(gpsData);

    // Append GPS data to CSV line
    dataString += gpsData;
  }

  // Write data line to SD card if data is not empty
  if (dataString.length() > 0) {
    dataFile.println(dataString);
    dataFile.flush();  // Ensure data is written to the SD card immediately
  }

  delay(100);  // Adjust delay as needed for data rate
}

void closeFile() {
  if (dataFile) {
    dataFile.close();  // Close file to ensure data integrity
  }
}
