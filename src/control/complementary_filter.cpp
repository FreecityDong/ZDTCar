#include "control/complementary_filter.h"

void ComplementaryFilter::reset(const float angleDeg) { angleDeg_ = angleDeg; }

float ComplementaryFilter::update(const float accelAngleDeg, const float gyroDegPerSec,
                                  const float dtSec) {
  angleDeg_ = kAlpha * (angleDeg_ + gyroDegPerSec * dtSec) + (1.0f - kAlpha) * accelAngleDeg;
  return angleDeg_;
}
