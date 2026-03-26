#include "control/balance_controller.h"

#include "common/math_utils.h"

void BalanceController::setGains(const PIDGains& gains) { gains_ = gains; }

void BalanceController::reset() { integral_ = 0.0f; }

float BalanceController::update(const float pitchDeg, const float targetPitchDeg,
                                const float gyroDegPerSec, const float dtSec) {
  const float error = targetPitchDeg - pitchDeg;
  integral_ += error * dtSec;
  integral_ = clampValue(integral_, -gains_.integralLimit, gains_.integralLimit);

  float output = gains_.kp * error + gains_.ki * integral_ - gains_.kd * gyroDegPerSec;
  output = clampValue(output, -gains_.outputLimit, gains_.outputLimit);
  return output;
}
