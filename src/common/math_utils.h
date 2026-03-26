#pragma once

#include <Arduino.h>

template <typename T>
constexpr T clampValue(const T value, const T minValue, const T maxValue) {
  return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

template <typename T>
constexpr T absValue(const T value) {
  return value < static_cast<T>(0) ? -value : value;
}

inline float lerpValue(const float from, const float to, const float alpha) {
  return from + (to - from) * alpha;
}
