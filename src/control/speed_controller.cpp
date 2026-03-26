#include "control/speed_controller.h"

#include "common/math_utils.h"

void SpeedController::setGains(const PIDGains& gains) { gains_ = gains; }

void SpeedController::reset() { integral_ = 0.0f; }

float SpeedController::update(const float targetRpm, const float actualRpm, const float dtSec) {
  const float error = targetRpm - actualRpm;
  integral_ += error * dtSec;
  integral_ = clampValue(integral_, -gains_.integralLimit, gains_.integralLimit);

  float output = gains_.kp * error + gains_.ki * integral_;
  output = clampValue(output, -gains_.outputLimit, gains_.outputLimit);
  return output;
}
