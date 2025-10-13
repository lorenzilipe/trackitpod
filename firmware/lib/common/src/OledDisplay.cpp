#include "common/OledDisplay.h"

namespace trackit {

OledDisplay::OledDisplay(uint8_t width, uint8_t height, TwoWire* wire, int8_t resetPin)
    : display_(width, height, wire, resetPin) {}

bool OledDisplay::begin(uint32_t i2cFreq) {
    if (!display_.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false)) {
        return false;
    }
    display_.display();
    display_.clearDisplay();
    display_.setTextSize(1);
    display_.setTextColor(SSD1306_WHITE);
    return true;
}

void OledDisplay::showSplash(const char* headline) {
    display_.clearDisplay();
    display_.setCursor(0, 0);
    display_.println(F("TRACKIT POD"));
    display_.println(headline);
    display_.display();
}

void OledDisplay::showTrackingStatus(const GeoCoordinate& base, const GeoCoordinate& tag, const AimSolution& aim) {
    display_.clearDisplay();
    display_.setCursor(0, 0);
    display_.print(F("Base: "));
    display_.print(base.latitudeDeg, 4);
    display_.print(F(","));
    display_.println(base.longitudeDeg, 4);
    display_.print(F("Tag:  "));
    display_.print(tag.latitudeDeg, 4);
    display_.print(F(","));
    display_.println(tag.longitudeDeg, 4);
    display_.print(F("Yaw:"));
    display_.print(aim.bearingDeg, 1);
    display_.print(F(" Elev:"));
    display_.println(aim.elevationDeg, 1);
    display_.display();
}

void OledDisplay::showTagStatus(const GeoCoordinate& tag, uint32_t sequence) {
    display_.clearDisplay();
    display_.setCursor(0, 0);
    display_.println(F("TAG MODE"));
    display_.print(F("Seq: "));
    display_.println(sequence);
    display_.print(F("Lat: "));
    display_.println(tag.latitudeDeg, 5);
    display_.print(F("Lon: "));
    display_.println(tag.longitudeDeg, 5);
    display_.display();
}

void OledDisplay::showCalibrationPrompt() {
    display_.clearDisplay();
    display_.setCursor(0, 0);
    display_.println(F("Calibrate"));
    display_.println(F("1. Level tripod"));
    display_.println(F("2. Aim front north"));
    display_.println(F("Press BTN"));
    display_.display();
}

void OledDisplay::showCalibrationComplete(float headingOffsetDeg) {
    display_.clearDisplay();
    display_.setCursor(0, 0);
    display_.println(F("Calibration"));
    display_.print(F("Offset: "));
    display_.println(headingOffsetDeg, 1);
    display_.display();
}

}  // namespace trackit
