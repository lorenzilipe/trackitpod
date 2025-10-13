#include "common/MotorController.h"

namespace trackit {

MotorController::MotorController(int panIn1, int panIn2, int panIn3, int panIn4,
                                 int tiltIn1, int tiltIn2, int tiltIn3, int tiltIn4,
                                 int stepsPerRevolution)
    : pan_(stepsPerRevolution, panIn1, panIn3, panIn2, panIn4),
      tilt_(stepsPerRevolution, tiltIn1, tiltIn3, tiltIn2, tiltIn4),
      stepsPerRevolution_(stepsPerRevolution) {}

void MotorController::begin() {
    pan_.setSpeed(10);  // rpm, adjust after testing
    tilt_.setSpeed(10);
}

void MotorController::zeroPosition() {
    currentPanDeg_ = 0.0f;
    currentTiltDeg_ = 0.0f;
}

void MotorController::moveToAngles(float targetPanDeg, float targetTiltDeg) {
    moveStepper(pan_, currentPanDeg_, targetPanDeg);
    moveStepper(tilt_, currentTiltDeg_, targetTiltDeg);
}

void MotorController::moveStepper(Stepper& motor, float& currentDeg, float targetDeg) {
    const float deltaDeg = targetDeg - currentDeg;
    if (fabs(deltaDeg) < 0.5f) {
        return;  // deadband to avoid unnecessary micro-movements
    }
    const long stepsToMove = static_cast<long>((deltaDeg / 360.0f) * stepsPerRevolution_);
    motor.step(stepsToMove);
    currentDeg += (static_cast<float>(stepsToMove) / stepsPerRevolution_) * 360.0f;
}

}  // namespace trackit
