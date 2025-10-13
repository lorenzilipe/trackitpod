#pragma once

#include <Stepper.h>

namespace trackit {

// Simple position keeper for the two 28BYJ-48 steppers controlling pan/tilt.
class MotorController {
  public:
    MotorController(int panIn1, int panIn2, int panIn3, int panIn4,
                    int tiltIn1, int tiltIn2, int tiltIn3, int tiltIn4,
                    int stepsPerRevolution = 2048);

    void begin();

    // Move steppers to the requested angles (degrees). This naive implementation assumes
    // that the zero position is the physical front of the rig after calibration.
    void moveToAngles(float targetPanDeg, float targetTiltDeg);

    float currentPanDeg() const { return currentPanDeg_; }
    float currentTiltDeg() const { return currentTiltDeg_; }

    // Resets the internal position reference to zero after manual calibration.
    void zeroPosition();

  private:
    Stepper pan_;
    Stepper tilt_;
    int stepsPerRevolution_;
    float currentPanDeg_ = 0.0f;
    float currentTiltDeg_ = 0.0f;

    void moveStepper(Stepper& motor, float& currentDeg, float targetDeg);
};

}  // namespace trackit
