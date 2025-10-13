#pragma once

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "types.h"

namespace trackit {

class OledDisplay {
  public:
    OledDisplay(uint8_t width, uint8_t height, TwoWire* wire, int8_t resetPin);

    bool begin(uint32_t i2cFreq = 400000);
    void showSplash(const char* headline);
    void showTrackingStatus(const GeoCoordinate& base, const GeoCoordinate& tag, const AimSolution& aim);
  void showTagStatus(const GeoCoordinate& tag, uint32_t sequence);
    void showCalibrationPrompt();
    void showCalibrationComplete(float headingOffsetDeg);

  private:
    Adafruit_SSD1306 display_;
};

}  // namespace trackit
