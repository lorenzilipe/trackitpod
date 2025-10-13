#include "common/TrackingMath.h"

#include <math.h>

namespace trackit {

AimSolution computeAimSolution(const GeoCoordinate& base, const GeoCoordinate& tag, float headingOffsetDeg) {
    AimSolution solution;

    if (!base.valid || !tag.valid) {
        return solution;  // invalid -> solution.valid remains false
    }

    const float lat1 = degToRad(base.latitudeDeg);
    const float lat2 = degToRad(tag.latitudeDeg);
    const float dLon = degToRad(tag.longitudeDeg - base.longitudeDeg);

    const float y = sinf(dLon) * cosf(lat2);
    const float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);
    float bearingDeg = fmodf(radToDeg(atan2f(y, x)) + 360.0f, 360.0f);
    bearingDeg += headingOffsetDeg;
    if (bearingDeg >= 360.0f) {
        bearingDeg -= 360.0f;
    }

    // Elevation estimate: use simple difference in altitude over ground distance.
    // For surfing use-case altitudes are near zero, but we keep this for completeness.
    const float earthRadiusM = 6371000.0f;
    const float centralAngle = acosf(sinf(lat1) * sinf(lat2) + cosf(lat1) * cosf(lat2) * cosf(dLon));
    const float groundDistance = earthRadiusM * centralAngle;
    float elevationDeg = 0.0f;
    if (groundDistance > 1.0f) {
        elevationDeg = radToDeg(atan2f(tag.altitudeM - base.altitudeM, groundDistance));
    }

    solution.bearingDeg = bearingDeg;
    solution.elevationDeg = elevationDeg;
    solution.valid = true;
    return solution;
}

}  // namespace trackit
