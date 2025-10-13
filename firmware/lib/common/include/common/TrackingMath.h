#pragma once

#include "types.h"

namespace trackit {

// Computes the great-circle bearing and elevation between the base and the tag.
AimSolution computeAimSolution(const GeoCoordinate& base, const GeoCoordinate& tag, float headingOffsetDeg = 0.0f);

// Converts degrees to radians.
inline float degToRad(float deg) { return deg * 0.01745329252f; }

// Converts radians to degrees.
inline float radToDeg(float rad) { return rad * 57.295779513f; }

}  // namespace trackit
