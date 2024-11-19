#include <TinyGPS++.h>

TinyGPSPlus gps; // This is the GPS object that will handle all the NMEA data

void setup()
{
  Serial.begin(9600);        // Initialize Serial Monitor
  Serial1.begin(9600);       // Initialize GPS Serial (connect GPS TX to Teensy RX1 and GPS RX to Teensy TX1)
  Serial.println("GPS Start"); // Just show to the monitor that the sketch has started
}

void loop()
{
  while(Serial1.available())  // While there are characters coming from the GPS
  {
    gps.encode(Serial1.read()); // Feed the serial NMEA data into the library one character at a time

    if(gps.location.isUpdated()) // Check if location data is updated
    {
      // Get the latest info from the GPS object
      Serial.print("Satellite Count: ");
      Serial.println(gps.satellites.value());

      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);

      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);

      Serial.print("Speed MPH: ");
      Serial.println(gps.speed.mph());

      Serial.print("Altitude Feet: ");
      Serial.println(gps.altitude.feet());

      Serial.println();
    }
  }
}
