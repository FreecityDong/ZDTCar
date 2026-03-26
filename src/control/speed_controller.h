#pragma once

#include "common/app_types.h"

class SpeedController {
 public:
  void setGains(const PIDGains& gains);
  void reset();
  float update(float targetRpm, float actualRpm, float dtSec);

 private:
  PIDGains gains_;
  float integral_ = 0.0f;
};
