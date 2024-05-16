/**
*
*
*/

//========================

// Import libraries
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Define data package structure. This is what gets sent over radio
struct DataPackage {
  double latitude;          // 4 bytes
  double longitude;         // 4
  short altitude;           // 2 (short is used for small integers in [-32768, 32767])
  byte numSatellites;       // 1
  float speedMps;           // 4
  float courseDegrees;      // 4
  byte padding[10];         // 10
                            //------
                            // 29
};

// Define constants
#define RADIO_TX_PIN 2 // Tx pin on radio, supposed to be used on RX field of SoftwareSerial object
#define RADIO_RX_PIN 3 // Rx pin on radio, supposed to be used on TX field of SoftwareSerial object
#define RADIO_BAUD_RATE 9600 // Baud rate for radio
#define GPS_TX_PIN 11 // Tx pin on gps, supposed to be used on RX field of SoftwareSerial object
#define GPS_RX_PIN 12 // Rx pin on gps, supposed to be used on TX field of SoftwareSerial object
#define GPS_BAUD_RATE 9600 // Baud rate for gps
#define MIN_GPS_SATELLITES_FOR_TRANSMISSION 2 // Minimum number of satellites fixed in order to transmit gps data

// Initialize instances
DataPackage gpsDataPackage; // DataPackage instance to store and send gps data
SoftwareSerial radioSerial(RADIO_TX_PIN, RADIO_RX_PIN); // SoftwareSerial object to communicate with radio
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN); // SoftwareSerial object to communicate with GPS
TinyGPSPlus gpsPlus; // TinyGPSPlus object to parse NMEA messages

// Initialize variables
bool newTxData = false; // Variable to keep track of whether new gps data has been updated
const byte startMarker = 255; // Start marker is sent before sending any new gps data. Base will only receive new data if it first receives the start marker.
const byte dataPackageSize = sizeof(gpsDataPackage); // Length of data package to be sent and expected by base

// Initialize timing variables for sending frequency
unsigned long prevUpdateTime = 0;
unsigned long updateInterval = 1000; // Minimum elapsed time in milliseconds before a new radio message is transmitted

//========================
// SETUP AND MAIN LOOP

void setup() {
  Serial.println("\nStarting setup for tag...\n"); // Print setup message
  Serial.begin(9600); // Start serial communication for debugging
  radioSerial.begin(RADIO_BAUD_RATE); // Start serial communication with radio module
  gpsSerial.begin(GPS_BAUD_RATE); // Start serial communication with gps module
}

void loop() {
  updateGpsData();
  transmitGpsData();
  //debugPrintNumSatellitesGPS();
}

//========================
// HELPER FUNCTIONS

/**
* This function checks that last GPS data has been sent and then attempts to read data from GPS if it is available. 
* If so, the data is checked to be valid and updated, and then updates the gpsDataPackage global object.
*/
void updateGpsData() {
  if (newTxData == false) { // Ensure previous message has been sent
    while (gpsSerial.available() > 0) { // While GPS has data ready at serial communication
      if (gpsPlus.encode(gpsSerial.read())) { // Verify that GPS data is valid
        if (gpsPlus.location.isUpdated()) { // Verify that GPS location was updated
          // Update data package to be sent with most current GPS data:
          gpsDataPackage.latitude = gpsPlus.location.lat();
          gpsDataPackage.longitude = gpsPlus.location.lng();
          gpsDataPackage.altitude = gpsPlus.altitude.meters();
          gpsDataPackage.numSatellites = gpsPlus.satellites.value();
          gpsDataPackage.speedMps = gpsPlus.speed.mps();
          gpsDataPackage.courseDegrees = gpsPlus.course.deg();
          newTxData = true; // New gps data acquired, not yet sent
        }
      }
    }
  }
}

/**
* This function checks that new GPS data has been gathered and then sends it to the radio. It will never send more than one message per updateInterval.
* debugPrintSentData is called.
*/
void transmitGpsData() {
  if (millis() - prevUpdateTime >= updateInterval) { // Timer to ensure that no more than one message is sent per updateInterval
    prevUpdateTime = millis(); // Timer-related stuff
    if (newTxData == true) {// && gpsDataPackage.numSatellites >= MIN_GPS_SATELLITES_FOR_TRANSMISSION) { // Ensure new gps data has been gathered and that number of satellites >= 2 before sending new message
      int bytesSent = radioSerial.write(startMarker); // Initialize variable to track # of bytes sent and send the start marker to radio
      bytesSent += radioSerial.write((byte*) &gpsDataPackage, dataPackageSize); // Send GPS data package, updating # of bytes sent along the way
      newTxData = false; // Update boolean to say that data has been sent and new gps data should be gathered
      debugPrintSentData(gpsDataPackage, bytesSent);
    }
  }
}

//========================
// DEBUG/PRINT FUNCTIONS

/**
* 
*/
void debugPrintNumSatellitesGPS() {
  Serial.print("\n====================\nSTART debugPrintNumSatellitesGPS\n====================\n");
  while (gpsSerial.available() > 0) {
    if (gpsPlus.encode(gpsSerial.read())) {
      Serial.println("GPS encoded");
      Serial.print("NumSatellites: ");
      Serial.println(gpsPlus.satellites.value(), 6);
      if (gpsPlus.location.isUpdated()) {
        Serial.println("GPS location is updated");
      }
      if (gpsPlus.location.isValid()) {
        Serial.println("GPS location is valid");
      }
    }
  }
  Serial.print("\n====================\nEND debugPrintNumSatellitesGPS\n====================\n");
}

/**
* This function prints the data currently stored in the gpsDataPackage object by accessing it from the global variables.
*/
void debugPrintCurrentGpsData() {
  Serial.print("\n====================\nSTART debugPrintCurrentGpsData\n====================\n");
  Serial.print("Sent: "); 
  Serial.print("Latitude: ");
  Serial.print(gpsDataPackage.latitude);
  Serial.print(", Longitude: ");
  Serial.print(gpsDataPackage.longitude);
  Serial.print(", Altitude: ");
  Serial.print(gpsDataPackage.altitude);
  Serial.print(", NumSatellites: ");
  Serial.print(gpsDataPackage.numSatellites);
  Serial.print(", SpeedMps: ");
  Serial.print(gpsDataPackage.speedMps);
  Serial.print(", CourseDegrees: ");
  Serial.print(gpsDataPackage.courseDegrees);
  Serial.print("\n====================\nEND debugPrintCurrentGpsData\n====================\n");
}

/**
* This function prints the data stored in the DataPackage object passed to it and the number of bytes sent (also must be passed to it).
*/
void debugPrintSentData(DataPackage sent, int bytesSent) {
  Serial.print("\n====================\nSTART debugPrintSentData\n====================\n");
  Serial.print("Sent: "); 
  Serial.print("Latitude: ");
  Serial.print(sent.latitude);
  Serial.print(", Longitude: ");
  Serial.print(sent.longitude);
  Serial.print(", Altitude: ");
  Serial.print(sent.altitude);
  Serial.print(", NumSatellites: ");
  Serial.print(sent.numSatellites);
  Serial.print(", SpeedMps: ");
  Serial.print(sent.speedMps);
  Serial.print(", CourseDegrees: ");
  Serial.print(sent.courseDegrees);
  Serial.println();
  Serial.print("Sent ");
  Serial.print(bytesSent);
  Serial.print(" bytes in total.");
  Serial.print("\n====================\nEND debugPrintSentData\n====================\n");
}