#include <Arduino.h>

#include <Wire.h>

#include "common/GpsModule.h"
#include "common/MotorController.h"
#include "common/OledDisplay.h"
#include "common/RfLink.h"
#include "common/TrackingMath.h"

using trackit::AimSolution;
using trackit::GeoCoordinate;
using trackit::GpsModule;
using trackit::MotorController;
using trackit::OledDisplay;
using trackit::RfLink;
using trackit::TrackingPacket;

// -------- Pin assignments ----------------------------------------------------
// These pin numbers follow Seeed Studio's XIAO ESP32-C3 pinout.
// Adjust them if you rewire the harness.
static constexpr int kGpsRxPin = 7;  // MCU RX <- GPS TX
static constexpr int kGpsTxPin = 6;  // MCU TX -> GPS RX (rarely used, but wired for completeness)
static constexpr int kRfRxPin = 5;   // MCU RX <- HC-12 TX
static constexpr int kRfTxPin = 4;   // MCU TX -> HC-12 RX
static constexpr int kCalibrationButtonPin = 2;

// Stepper driver pins (ULN2003). Reorder to match your wiring.
static constexpr int kPanIn1 = 10;
static constexpr int kPanIn2 = 9;
static constexpr int kPanIn3 = 8;
static constexpr int kPanIn4 = 3;
static constexpr int kTiltIn1 = 1;
static constexpr int kTiltIn2 = 0;
static constexpr int kTiltIn3 = 21;
static constexpr int kTiltIn4 = 20;

static constexpr uint32_t kDisplayI2CFrequency = 400000;  // 400 kHz fast mode
static constexpr uint32_t kGpsBaud = 9600;
static constexpr uint32_t kRfBaud = 9600;

// -------- Hardware abstraction layers ---------------------------------------
HardwareSerial GPSPort(1);   // UART1 for GPS
HardwareSerial RadioPort(2); // UART2 for HC-12. If unavailable, reassign pins and index.
GpsModule baseGps(GPSPort);
RfLink radio(RadioPort);
MotorController motors(kPanIn1, kPanIn2, kPanIn3, kPanIn4,
                       kTiltIn1, kTiltIn2, kTiltIn3, kTiltIn4);
OledDisplay display(128, 64, &Wire, -1);

// -------- Tracking state -----------------------------------------------------
TrackingPacket latestTagPacket;
unsigned long lastPacketMs = 0;
float headingOffsetDeg = 0.0f;  // Stores mechanical zero -> geographic north offset
bool calibrationDone = false;

enum class SystemState { kAwaitingCalibration, kReadyToTrack };
SystemState state = SystemState::kAwaitingCalibration;

// Forward declarations
void attemptCalibration(const GeoCoordinate& baseFix);
float wrapAngle(float angleDeg);

void setup() {
    pinMode(kCalibrationButtonPin, INPUT_PULLUP);

    Serial.begin(115200);
    GPSPort.begin(kGpsBaud, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
    RadioPort.begin(kRfBaud, SERIAL_8N1, kRfRxPin, kRfTxPin);

    baseGps.begin(kGpsBaud);
    radio.begin(kRfBaud);
    motors.begin();

    Wire.begin();
    display.begin(kDisplayI2CFrequency);
    display.showSplash("BASE BOOTING");

    Serial.println(F("TrackItPod base station starting..."));
}

void loop() {
    // 1. Continuously feed incoming GPS data.
    baseGps.processIncomingData();
    const GeoCoordinate baseFix = baseGps.latestFix();

    // 2. Listen for HC-12 packets from the tag.
    TrackingPacket packet;
    if (radio.receivePacket(&packet)) {
        latestTagPacket = packet;
        lastPacketMs = millis();
        Serial.print(F("Tag packet seq: "));
        Serial.println(packet.sequence);
    }

    // 3. If the user holds the calibration button, align the mechanical front.
    if (digitalRead(kCalibrationButtonPin) == LOW) {
        attemptCalibration(baseFix);
    }

    if (state == SystemState::kReadyToTrack) {
        const unsigned long staleThresholdMs = 2000;
        const bool packetIsFresh = (millis() - lastPacketMs) < staleThresholdMs;
        if (packetIsFresh) {
            AimSolution aim = trackit::computeAimSolution(baseFix, latestTagPacket.tagPosition, -headingOffsetDeg);
            if (aim.valid) {
                const float targetPan = wrapAngle(aim.bearingDeg);
                const float targetTilt = constrain(aim.elevationDeg, -30.0f, 45.0f);
                motors.moveToAngles(targetPan, targetTilt);
                display.showTrackingStatus(baseFix, latestTagPacket.tagPosition, aim);
            }
        } else {
            display.showSplash("WAITING TAG");
        }
    } else {
        display.showCalibrationPrompt();
    }
}

void attemptCalibration(const GeoCoordinate& baseFix) {
    display.showSplash("CALIBRATING");

    if (!baseFix.valid) {
        Serial.println(F("Base GPS invalid – waiting for fix."));
        display.showSplash("BASE GPS ???");
        delay(250);
        return;
    }

    const unsigned long staleThresholdMs = 2000;
    if ((millis() - lastPacketMs) > staleThresholdMs || !latestTagPacket.tagPosition.valid) {
        Serial.println(F("No fresh tag data for calibration."));
        display.showSplash("TAG GPS ???");
        delay(250);
        return;
    }

    // The motors should still be at zero (pointing to the rig's mechanical front).
    // We compute the bearing to the tag and record it as our offset.
    AimSolution rawAim = trackit::computeAimSolution(baseFix, latestTagPacket.tagPosition);
    if (!rawAim.valid) {
        Serial.println(F("Aim solution invalid during calibration."));
        return;
    }

    headingOffsetDeg = rawAim.bearingDeg;
    motors.zeroPosition();
    calibrationDone = true;
    state = SystemState::kReadyToTrack;
    display.showCalibrationComplete(headingOffsetDeg);
    Serial.print(F("Calibration complete. Heading offset: "));
    Serial.println(headingOffsetDeg);
    delay(500);
}

float wrapAngle(float angleDeg) {
    float wrapped = fmod(angleDeg, 360.0f);
    if (wrapped > 180.0f) {
        wrapped -= 360.0f;
    }
    if (wrapped < -180.0f) {
        wrapped += 360.0f;
    }
    return wrapped;
}
