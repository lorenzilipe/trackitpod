/**
*
*
*/

//========================

// Import libraries
#include <math.h>
#include <SoftwareSerial.h>
#include <Stepper.h>

// Define data package structure. This is what gets sent/received over radio
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

// Function to convert degrees of latitude and longitude to radians
double toRadians(double degree) {
    return degree * (M_PI / 180.0);
}

DataPackage tagData;

// Define constants
#define RADIO_TX_PIN 2 // Tx pin on radio, supposed to be used on RX field of SoftwareSerial object
#define RADIO_RX_PIN 3 // Rx pin on radio, supposed to be used on TX field of SoftwareSerial object
#define RADIO_BAUD_RATE 9600 // Baud rate for radio
#define GPS_TX_PIN 5 // Tx pin on gps, supposed to be used on RX field of SoftwareSerial object
#define GPS_RX_PIN 6 // Rx pin on gps, supposed to be used on TX field of SoftwareSerial object
#define GPS_BAUD_RATE 9600 // Baud rate for gps
#define PAN_IN1 4 // Pan motor pin 1
#define PAN_IN2 5 // Pan motor pin 2
#define PAN_IN3 6 // Pan motor pin 3
#define PAN_IN4 7 // Pan motor pin 4
#define TILT_IN1 8 // Tilt motor pin 1
#define TILT_IN2 9 // Tilt motor pin 2
#define TILT_IN3 10 // Tilt motor pin 3
#define TILT_IN4 11 // Tilt motor pin 4
#define STEPS_PER_REVOLUTION 2048 // Number of steps for one revolution for stepper motor (0.18 deg per step for 28BYJ-48)
#define STEPPER_SPEED 15 // Stepper motor speed (max speed for 28BYJ-48 = 15)
#define CALIBRATION_POTENTIOMETER_PIN A7 // (TEMPORARY) Analog pin for calibration potentiometer

bool newTagData = false; // Variable to keep track of whether new gps data has been updated
const byte startMarker = 255;
const byte tagDataLen = sizeof(tagData);

// Define variables
float calibrationPot;
float baseLongitude, baseLatitude, baseAltitude;
float calibrationLongitude, calibrationLatitude, calibrationAltitude;
float previousLongitude, previousLatitude, previousAltitude;
float calibrationBaseTagDistance, baseTagDistance;
float leftoverStepsPan, leftoverStepsTilt; 
bool dataReceived;
float trackingLongitude, trackingLatitude, trackingAltitude, panAngle, tiltAngle;
float panAngleFromStart, tiltAngleFromStart;
int panSteps, tiltSteps;
float aDistPan, cDistPan, anglePan;
int roundedStepsPan;
// Radian variables
float baseLongitudeRad, baseLatitudeRad;
float calibrationLongitudeRad, calibrationLatitudeRad;
float previousLongitude, previousLatitudeRad;


// Initialize instances
DataPackage gpsDataPackage; // DataPackage instance to store and send gps data
SoftwareSerial radioSerial(RADIO_TX_PIN, RADIO_RX_PIN); // Radio instance to initialize radio module
Stepper panStepper(STEPS_PER_REVOLUTION, PAN_IN1, PAN_IN3, PAN_IN2, PAN_IN4); // Motor instance for the PAN motor (x)
//Stepper tiltStepper(STEPS_PER_REVOLUTION, TILT_IN1, TILT_IN3, TILT_IN2, TILT_IN4); // Motor instance for the TILT motor (y)
//LiquidCrystal_I2C lcd(0x27, 16, 2); // (TEMPORARY) Set the LCD address to 0x27 for a 16 chars and 2 line display

//========================
// SETUP AND MAIN LOOP

void setup() {
  Serial.begin(9600); // Initiate serial communication to see messages on serial monitor
  panStepper.setSpeed(STEPPER_SPEED); // Set stepper to max RPM
  pinMode(CALIBRATION_POTENTIOMETER_PIN, INPUT); // (TEMPORARY) Initialize the calibration potentiometer as anf input
  radioSerial.begin(RADIO_BAUD_RATE); // Start radio
	//lcd.begin(16, 2); // (TEMPORARY) Initialize the LCD
	//lcd.backlight(); // (TEMPORARY) Turn on LCD backlight


}

void loop() {
  calibrationPot = analogRead(CALIBRATION_POTENTIOMETER_PIN);
  receiveTagData(); // Will update the BASE with the new TAG data
  if (newTagData == true) { // This bit checks if a message has been received
    showNewTagData();
    newTagData = false; // Updates to false to await new data
  }
  if (calibrationPot <= 20) {
    calibrationStateOne();
    delay(100);
  } else if (calibrationPot <= 600) {
    calibrationBaseTagDistance = calibrationStateTwo();
    delay(100);
  } else {
    // Regular State
    trackingState();
    delay(100);
  }
  dataReceived = false;
}

//========================
// HELPER FUNCTIONS

// Loop to receive the data from the TAG
void receiveTagData() {
  // Initialize variables
  byte receivedByte;
  byte * structStart;
  structStart = reinterpret_cast <byte *> (&tagData);

  if (radioSerial.available() >= tagDataLen + 1 and newTagData == false) {
    dataReceived = true;
    receivedByte = radioSerial.read();
    if (receivedByte == startMarker) {
      // copy the bytes to the struct
      for (byte n = 0; n < tagDataLen; n++) {
        *(structStart + n) = radioSerial.read();
      }
      // make sure there is no garbage left in the buffer
      while (radioSerial.available() > 0) {
        byte dumpTheData = radioSerial.read();
      }

      newTagData = true;
    }
  }
}

// Loop to establish first step of calibration (TAG on top of BASE)
void calibrationStateOne() {

  // Read values from radio receiver
  baseLongitude = tagData.longitude;
  baseLatitude = tagData.latitude;
  baseAltitude = tagData.altitude;

  // (DEBUGGING) Print values on the serial monitor
  showCalibrationOneData(baseLongitude, baseLatitude, baseAltitude);
}

// Function to establish origin angle between BASE and TAG (TAG away from BASE)
float calibrationStateTwo() {
  Serial.println("---");
  Serial.println("Calibration state 2...");

  // Set totalPanAngle to 0 (track total angle from CS2 final position for debugging)
  float totalPanAngle = 0;

  // Read values from radio receiver
  calibrationLongitude = tagData.longitude;
  calibrationLatitude = tagData.latitude;
  calibrationAltitude = tagData.altitude;

  // Calculate distance between TAG and BASE (a)
  calibrationBaseTagDistance = sqrt(pow(calibrationLongitude - baseLongitude, 2) + pow(calibrationLatitude - baseLatitude, 2));

  // Update values for next iteration
  previousLongitude = calibrationLongitude;
  previousLatitude = calibrationLatitude;
  previousAltitude = calibrationAltitude;
  
  // (DEBUGGING) Print values on the serial monitor
  showCalibrationTwoData(calibrationLongitude, calibrationLatitude, calibrationAltitude, calibrationBaseTagDistance);

  return calibrationBaseTagDistance;
}

// CALCULATEPANANGLE USES BASETAGDISTANCE, WHICH IS ONLY CALCULATED IN CALIB STATE 2 AND NOT IN TRACKING STATE
// Refazer o codigo de calcular angulo usando triangulo maior e guardando angulo total entre CS2 final e current tracking position
// Potenciometro na tag pra mudar de calibration e tracking state (ou usar um botao logo) - so vale a pena quando tiver telinha na tag

// Function to keep tracking the tag after all calibration has been made
void trackingState() {

  // Read values from radio receiver
  trackingLongitude = tagData.longitude;
  trackingLatitude = tagData.latitude;
  trackingAltitude = tagData.altitude;
  
  //Calculate angles to move the motor by
  panAngle = calculatePanAngle(previousLongitude, previousLatitude, trackingLongitude, trackingLatitude, baseTagDistance);
  panSteps = angleToSteps(panAngle);
  //tiltAngle = calculateTiltAngle(previousLongitude, previousLatitude, trackingLongitude, trackingLatitude, baseTagDistance); needs previousAltitude and trackingAltitude as well as baseAltitude
  //tiltSteps = angleToSteps(tiltAngle);
  
  //Move the pan and tilt motors accordingly
  panStepper.step(panSteps); // tell stepper to move desired number of steps
  //tiltStepper.step(tiltSteps); // tell stepper to move desired number of steps
  
  //Calculate the angle from the original calibration 2 position
  panAngleFromStart = calculatePanAngle(calibrationLongitude, calibrationLatitude, trackingLongitude, trackingLatitude, baseTagDistance);

  previousLongitude = trackingLongitude;
  previousLatitude = trackingLatitude;

  // (DEBUGGING) Print values on the serial monitor
  showTrackingData(trackingLongitude, trackingLatitude, trackingAltitude, panAngle, panSteps);

  delay(100);
}

float calculatePanAngle(float previousLongitude, float previousLatitude, float trackingLongitude, float trackingLatitude, float baseTagDistance) {
  // calculate distance between TAG and BASE (b)
  bDistPan = sqrt(pow(trackingLongitude - baseLongitude, 2) + pow(trackingLatitude - baseLatitude, 2));
  // calculate distance between tag and previous tag position (c)
  cDistPan = sqrt(pow(previousLongitude - trackingLongitude, 2) + pow(previousLatitude - trackingLatitude, 2));
  // calculate distance between base and previous tag position (a)
  aDistPan = sqrt(pow(previousLongitude - baseLongitude, 2) + pow(previousLatitude - baseLatitude, 2));

  // calculate angle between previous tag position and new position
  if (trackingLatitude >= trackingLongitude * ((baseLatitude - previousLatitude) / (baseLongitude - previousLongitude))) {
    anglePanRad = -1 * (acos((pow(aDistPan, 2) + pow(baseTagDistance, 2) - pow(cDistPan, 2)) / (2 * aDistPan * baseTagDistance));
    anglePanDeg = anglePanRad * 180 / (atan(1) * 4))
  } else {
    anglePan = acos((pow(aDistPan, 2) + pow(baseTagDistance, 2) - pow(cDistPan, 2)) / (2 * aDistPan * baseTagDistance)) * 180 / (atan(1) * 4);
  }

  return anglePan;
}

int angleToSteps(float anglePan) {
  float stepsPan;
  stepsPan = anglePan / (360 / STEPS_PER_REVOLUTION);
  roundedStepsPan = round(stepsPan);
  leftoverStepsPan += stepsPan - roundedStepsPan;
  if (leftoverStepsPan >= 1) {
    roundedStepsPan += 1;
    leftoverStepsPan -= 1;
  } else if (leftoverStepsPan <= -1) {
    roundedStepsPan -= 1;
    leftoverStepsPan += 1;
  }
  return roundedStepsPan;
}

//========================
// DEBUG/PRINT FUNCTIONS

// Function to show new data received from the TAG
void showNewTagData() {
  Serial.println();
  Serial.print("Received: "); 
  Serial.print("Latitude: ");
  Serial.print(tagData.latitude);
  Serial.print(", Longitude: ");
  Serial.print(tagData.longitude);
  Serial.print(", Altitude: ");
  Serial.print(tagData.altitude);
  Serial.print(", NumSatellites: ");
  Serial.print(tagData.numSatellites);
  Serial.print(", SpeedMps: ");
  Serial.print(tagData.speedMps);
  Serial.print(", CourseDegrees: ");
  Serial.print(tagData.courseDegrees);
  Serial.println();
}

void showCalibrationOneData(float baseLongitude, float baseLatitude, float baseAltitude) {
  Serial.println("---");
  Serial.println("Calibration state 1...");
  Serial.println();
  Serial.print("BASE Latitude: ");
  Serial.print(baseLatitude, 6);
  Serial.print(", BASE Longitude: ");
  Serial.print(baseLongitude, 6);
  Serial.print(", BASE Altitude: ");
  Serial.print(baseAltitude, 3);
  Serial.println();
}

void showCalibrationTwoData(float calibrationLongitude, float calibrationLatitude, float calibrationAltitude, float calibrationBaseTagDistance) {
  Serial.println("---");
  Serial.println("Calibration state 2...");
  Serial.println();
  Serial.print("TAG Latitude: ");
  Serial.print(calibrationLatitude, 6);
  Serial.print(", TAG Longitude: ");
  Serial.print(calibrationLongitude, 6);
  Serial.print(", TAG Altitude: ");
  Serial.print(calibrationAltitude, 3);
  Serial.println();
  Serial.print("Calibration BASE/TAG Distance: ");
  Serial.print(calibrationBaseTagDistance, 5);
  Serial.println();
}

void showTrackingData(float trackingLongitude, float trackingLatitude, float trackingAltitude, float panAngle, int panSteps) {
  Serial.println("---");
  Serial.println("Tracking...");
  Serial.print("Longitude: ");
  Serial.print(trackingLongitude, 6);
  Serial.print(", Latitude: ");
  Serial.println(trackingLatitude, 6);
  Serial.print("Satellites: ");
  Serial.print(tagData.numSatellites);
  Serial.print(", Altitude: ");
  Serial.print(tagData.altitude);
  Serial.print(", Speed (m/s): ");
  Serial.print(tagData.speedMps);
  Serial.print(", Course (degrees): ");
  Serial.println(tagData.courseDegrees);
  Serial.print("Moving motor by ");
  Serial.print(panAngle, 5);
  Serial.print("degrees = ");
  Serial.print(panSteps);
  Serial.println("steps.");
}