#include <TinyGPS++.h>

TinyGPSPlus gps; // GPS object for handling NMEA data

void setup()
{
  Serial.begin(9600);        // Initialize Serial Monitor
  Serial1.begin(9600);       // Initialize GPS Serial (connect GPS TX to Teensy RX1 and GPS RX to Teensy TX1)
  Serial.println("GPS Start"); // Show that the sketch has started
  
  // Print CSV header for clarity
  Serial.println("Satellite_Count,Latitude,Longitude,Speed_MPH,Altitude_Feet");
}

void loop()
{
  while (Serial1.available()) // While there are characters coming from the GPS
  {
    gps.encode(Serial1.read()); // Feed the serial NMEA data into the library one character at a time

    if (gps.location.isUpdated()) // Check if location data is updated
    {
      // Print GPS data in CSV format
      Serial.print(gps.satellites.value());  // Satellite count
      Serial.print(",");

      Serial.print(gps.location.lat(), 6);   // Latitude
      Serial.print(",");

      Serial.print(gps.location.lng(), 6);   // Longitude
      Serial.print(",");

      Serial.print(gps.speed.mph());         // Speed in MPH
      Serial.print(",");

      Serial.println(gps.altitude.feet());   // Altitude in feet
    }
  }
}
