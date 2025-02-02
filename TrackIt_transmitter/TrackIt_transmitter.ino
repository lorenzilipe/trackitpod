#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <string.h>

struct Data_Package {
  float lat;
  float lng;
  float alt;
  int satellites;
  float speed;
  float course;
};

const byte address[6] = "00001"; // radio address
#define RADIO_TX_PIN 8 // Pin for CE on radio
#define RADIO_RX_PIN 9 // Pin for CSN on radio
#define RADIO_BAUD_RATE 9600 // Baud rate for radio
#define GPS_RX_PIN 4 // Pin for RX on GPS
#define GPS_TX_PIN 3 // Pin for TX on GPS
#define GPS_BAUD_RATE 9600 // Baud rate for GPS

SoftwareSerial radio(RADIO_TX_PIN, RADIO_RX_PIN); // Create a SoftwareSerial object to communicate with the radio
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // Create a SoftwareSerial object to communicate with the GPS sensor
TinyGPSPlus gps; // Create an instance of the TinyGPS++ object
Data_Package gpsData; // Create Data_Package object for storing and sending gps data

void setup() {
  // Initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  gpsSerial.begin(GPS_BAUD_RATE);
  radio.begin(RADIO_BAUD_RATE);

  // Default values for GPS
  gpsData.lat = gpsData.lng = gpsData.alt = gpsData.satellites = 9999;
}

void loop() {
  // Read the GPS sensor data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) { // Obtaining latitude and longitude values
        gpsData.lat = gps.location.lat();
        gpsData.lng = gps.location.lng();
      } else {
        gpsData.lat = 9999;
        gpsData.lng = 9999;
      }
      if (gps.altitude.isValid()) {
        gpsData.alt = gps.altitude.value(); // Obtaining altitude in centimeters
      } else {
        gpsData.alt = 9999;
      }
      if (gps.satellites.isValid()) {
        gpsData.satellites = gps.satellites.value(); // Obtaining the number of connected satellites
      } else {
        gpsData.satellites = 9999;
      }
      if (gps.speed.isValid()) {
        gpsData.speed = gps.speed.kmph(); // Obtaining the predicted speed in km/h
      } else {
        gpsData.speed = 9999;
      }
      if (gps.course.isValid()) {
        gpsData.course = gps.course.deg(); // Obtaining predicted angular direction in degrees
      } else {
        gpsData.course = 9999;
      }
      // Debug
      Serial.println("---");
      Serial.print("Latitude: ");
      Serial.println(gpsData.lat, 16);
      Serial.print("Longitude: ");
      Serial.println(gpsData.lng, 16);
      Serial.print("Altitude (cm): ");
      Serial.println(gpsData.alt, 16);
      Serial.print("Satellites: ");
      Serial.println(gpsData.satellites);
      Serial.print("Speed: ");
      Serial.println(gpsData.speed, 16);
      Serial.print("Course: ");
      Serial.println(gpsData.course, 16);
    }
  }
  // Transmit GPS sensor data
  int bytesSent = radioTransmit(radio, gpsData);
  Serial.println(std::format("Sent {} bytes.", bytesSent));
}

int radioTransmit(SoftwareSerial radio, DataPackage gpsData) {
  string fmt = "<{lat};{lng};{alt};{sat};{spd};{crs}>";
  string encoded = std::format(
    fmt, 
    gpsData.lat, 
    gpsData.lng, 
    gpsData.alt, 
    gpsData.satellites, 
    gpsData.speed, 
    gpsData.course
  );
  char data[] = new char[encoded.length() + 1];
  strcpy(data, encoded.c_str())
  return radio.write(encoded);
}