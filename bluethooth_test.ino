#include <SoftwareSerial.h>

// Define HC-06 connections
#define HC06_RX 34  // Teensy Pin RX2
#define HC06_TX 35 // Teensy Pin TX2

// Initialize a serial connection for HC-06
SoftwareSerial hc06(HC06_RX, HC06_TX);

void setup() {
  Serial.begin(115200);  // For Serial Monitor debugging
  hc06.begin(9600);      // HC-06 default baud rate is 9600
  delay(1000);

  Serial.println("HC-06 Module Ready");
  hc06.println("Bluetooth Connection Ready"); // Send an initial message to paired device
}

void loop() {
  // Send a test message to the HC-06
  hc06.println("Hello from the Teensy!");
  delay(1000); // 1-second interval for sending data

  // If data is received from the HC-06, print it to the Serial Monitor
  if (hc06.available()) {
    String received = hc06.readString();
    Serial.print("Received via Bluetooth: ");
    Serial.println(received);
  }
}
