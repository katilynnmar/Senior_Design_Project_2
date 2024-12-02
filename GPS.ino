#include <TinyGPS++.h>
#include <SD.h>
#include <SoftwareSerial.h>

#define HC06_RX 34              // Teensy Pin RX2
#define HC06_TX 35              // Teensy Pin TX2
#define SD_CS_PIN BUILTIN_SDCARD // Pin for the built-in SD card reader on Teensy


TinyGPSPlus gps; // This is the GPS object that will handle all the NMEA data
File dataFile;
SoftwareSerial hc06(HC06_RX, HC06_TX);

void setup()
{
  Serial.begin(9600);        // Initialize Serial Monitor

  Serial1.begin(9600);       // Initialize GPS Serial (connect GPS TX to Teensy RX1 and GPS RX to Teensy TX1)
  hc06.begin(9600);    // Bluetooth Serial
  Serial.println("GPS Start"); // Just show to the monitor that the sketch has started

  Serial.println("System initializing...");
  hc06.println("Bluetooth communication active.");

  // Wait for Serial Monitor
  while (!Serial) delay(10);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Failed to initialize SD card!");
    hc06.println("Failed to initialize SD card!");
    while (1);
  }

  dataFile = SD.open("gpslog.csv", FILE_WRITE); // Open the CSV file
  if (dataFile) {
    // Write CSV header
    dataFile.println("Satellite_Count,Latitude,Longitude,Speed_MPH,Altitude_Feet");
    dataFile.flush();
    Serial.println("SD card initialized. GPS data logging started.");
    hc06.println("SD card initialized. GPS data logging started.");
  } else {
    Serial.println("Error opening gpslog.csv!");
    hc06.println("Error opening gpslog.csv!");
    while (1);
  }
}

void loop()
{
  while(Serial1.available())  // While there are characters coming from the GPS
  {
    gps.encode(Serial1.read()); // Feed the serial NMEA data into the library one character at a time

    if(gps.location.isUpdated()) // Check if location data is updated
    {
      String gpsData = String(gps.satellites.value()) + "," +
                       String(gps.location.lat(), 6) + "," +
                       String(gps.location.lng(), 6) + "," +
                       String(gps.speed.mph()) + "," +
                       String(gps.altitude.feet());

      if (dataFile) {
        dataFile.println(gpsData);
        dataFile.flush(); // Ensure immediate write to the SD card
      } else {
        Serial.println("Error writing to gpslog.csv!");
        hc06.println("Error writing to gpslog.csv!");
      }

      Serial.println(gpsData);
      hc06.println(gpsData);
      // Get the latest info from the GPS object
      

      if (!Serial1.available()) {
        Serial.println("No GPS data available. Waiting for signal...");
        hc06.println("No GPS data available. Waiting for signal...");
      }

      Serial.println();
    }
  }
}

void closeFile() {
  if (dataFile) {
    dataFile.close(); // Close file to ensure data integrity
  }
}

