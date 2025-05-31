#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Create a TinyGPS++ object
TinyGPSPlus gps;

// Create a HardwareSerial object (UART2 on ESP32)
HardwareSerial GPS_Serial(2);

// Setup function
void setup() {
  Serial.begin(115200);           // Serial monitor baud rate
  GPS_Serial.begin(9600, SERIAL_8N1,35, 32); // GPS module baud rate, TX=35, RX=32 (you can change pins if needed)

  Serial.println("ESP32 GPS Tracker Initialized...");
}

// Loop function
void loop() {
  while (GPS_Serial.available() > 0) {
    char c = GPS_Serial.read();

    if (gps.encode(c)) {
      if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);

        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);

        Serial.print("Altitude (meters): ");
        Serial.println(gps.altitude.meters());

        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());

        Serial.print("Speed (km/h): ");
        Serial.println(gps.speed.kmph());

        Serial.println("--------------------------");
      } else {
        Serial.println("Waiting for valid GPS signal...");
      }
    }
  }
}
