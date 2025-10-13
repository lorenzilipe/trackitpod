#include <Arduino.h>
#include <Wire.h>

#include "common/GpsModule.h"
#include "common/OledDisplay.h"
#include "common/RfLink.h"

using trackit::GeoCoordinate;
using trackit::GpsModule;
using trackit::OledDisplay;
using trackit::RfLink;
using trackit::TrackingPacket;

static constexpr int kGpsRxPin = 7;
static constexpr int kGpsTxPin = 6;
static constexpr int kRfRxPin = 5;
static constexpr int kRfTxPin = 4;

static constexpr uint32_t kGpsBaud = 9600;
static constexpr uint32_t kRfBaud = 9600;
static constexpr uint32_t kBroadcastIntervalMs = 250;

HardwareSerial GPSPort(1);
HardwareSerial RadioPort(2);
GpsModule tagGps(GPSPort);
RfLink radio(RadioPort);
OledDisplay display(128, 64, &Wire, -1);

uint32_t sequenceCounter = 0;
unsigned long lastBroadcastMs = 0;

void setup() {
    Serial.begin(115200);

    GPSPort.begin(kGpsBaud, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
    RadioPort.begin(kRfBaud, SERIAL_8N1, kRfRxPin, kRfTxPin);

    tagGps.begin(kGpsBaud);
    radio.begin(kRfBaud);

    Wire.begin();
    display.begin();
    display.showSplash("TAG BOOTING");

    Serial.println(F("TrackItPod tag starting..."));
}

void loop() {
    tagGps.processIncomingData();
    GeoCoordinate fix = tagGps.latestFix();

    const unsigned long now = millis();
    if (fix.valid && (now - lastBroadcastMs) >= kBroadcastIntervalMs) {
        TrackingPacket packet;
        packet.sequence = ++sequenceCounter;
        packet.tagPosition = fix;
        radio.sendPacket(packet);
        lastBroadcastMs = now;
        Serial.print(F("Broadcasting seq: "));
        Serial.println(packet.sequence);
    }

    if (fix.valid) {
        display.showTagStatus(fix, sequenceCounter);
    } else {
        display.showSplash("ACQUIRING GPS");
    }
}
