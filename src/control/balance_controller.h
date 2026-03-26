#pragma once

#include "common/app_types.h"

class BalanceController {
 public:
  void setGains(const PIDGains& gains);
  void reset();
  float update(float pitchDeg, float targetPitchDeg, float gyroDegPerSec, float dtSec);

 private:
  PIDGains gains_;
  float integral_ = 0.0f;
};
