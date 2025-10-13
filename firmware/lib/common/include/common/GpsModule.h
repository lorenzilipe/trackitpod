#pragma once

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include "types.h"

namespace trackit {

// Thin wrapper around TinyGPSPlus that adds buffering and timestamps.
class GpsModule {
  public:
    explicit GpsModule(HardwareSerial& serialPort);

    // Configure the UART used by the GPS board.
    void begin(uint32_t baudRate = 9600);

    // Feed bytes from the GPS into TinyGPSPlus. Call this frequently in loop().
    void processIncomingData();

    // Returns the most recent parsed fix. "valid" will be false if no fix yet.
    GeoCoordinate latestFix() const { return latestFix_; }

  private:
    HardwareSerial& serial_;
    TinyGPSPlus parser_;
    GeoCoordinate latestFix_;
};

}  // namespace trackit
