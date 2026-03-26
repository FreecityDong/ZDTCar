#pragma once

class ComplementaryFilter {
 public:
  void reset(const float angleDeg = 0.0f);
  float update(float accelAngleDeg, float gyroDegPerSec, float dtSec);
  float angleDeg() const { return angleDeg_; }

 private:
  float angleDeg_ = 0.0f;
  static constexpr float kAlpha = 0.98f;
};
