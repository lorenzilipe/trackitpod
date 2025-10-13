#include "common/GpsModule.h"

namespace trackit {

GpsModule::GpsModule(HardwareSerial& serialPort) : serial_(serialPort) {}

void GpsModule::begin(uint32_t baudRate) {
    serial_.begin(baudRate);
}

void GpsModule::processIncomingData() {
    while (serial_.available()) {
        const char c = serial_.read();
        if (parser_.encode(c) && parser_.location.isValid()) {
            latestFix_.latitudeDeg = parser_.location.lat();
            latestFix_.longitudeDeg = parser_.location.lng();
            if (parser_.altitude.isValid()) {
                latestFix_.altitudeM = parser_.altitude.meters();
            }
            if (parser_.hdop.isValid()) {
                latestFix_.hdop = parser_.hdop.value() / 100.0f;  // convert from cm representation
            }
            latestFix_.timestampMs = millis();
            latestFix_.valid = true;
        }
    }
}

}  // namespace trackit
