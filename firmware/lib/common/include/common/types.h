#pragma once

#include <Arduino.h>

namespace trackit {

// Represents a position fix reported by a GPS receiver.
struct GeoCoordinate {
    float latitudeDeg = 0.0f;   // Latitude in decimal degrees (positive north of the equator)
    float longitudeDeg = 0.0f;  // Longitude in decimal degrees (positive east of Greenwich)
    float altitudeM = 0.0f;     // Altitude above mean sea level in meters (if available)
    float hdop = 999.0f;        // Horizontal dilution of precision (smaller is better)
    unsigned long timestampMs = 0;  // Milliseconds since boot when the fix was captured
    bool valid = false;         // True when the GPS provided a fix with usable data
};

// Single radio frame transmitted from the wearable tag to the base station.
struct TrackingPacket {
    uint32_t sequence = 0;      // Packet counter used for loss detection
    GeoCoordinate tagPosition;  // Latest position of the subject
};

// Holds the bearing (heading) and elevation that the camera rig should adopt.
struct AimSolution {
    float bearingDeg = 0.0f;    // Angle in degrees clockwise from geographic north
    float elevationDeg = 0.0f;  // Positive angles pitch the camera upward
    bool valid = false;         // False when we cannot compute a reliable solution yet
};

}  // namespace trackit
