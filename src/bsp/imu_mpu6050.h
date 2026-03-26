#pragma once

#include <Adafruit_MPU6050.h>
#include <Wire.h>

struct ImuSample {
  bool valid = false;
  float accelXMps2 = 0.0f;
  float accelYMps2 = 0.0f;
  float accelZMps2 = 0.0f;
  float gyroXDps = 0.0f;
  float gyroYDps = 0.0f;
  float gyroZDps = 0.0f;
  float pitchAccelDeg = 0.0f;
  uint32_t sampleTimeUs = 0;
};

class ImuMpu6050 {
 public:
  bool begin(TwoWire& wire, int sdaPin, int sclPin);
  bool calibrateGyro(size_t sampleCount = 500);
  bool update();
  const ImuSample& sample() const { return sample_; }
  uint32_t lastUpdateMs() const { return lastUpdateMs_; }
  bool isOnline() const { return online_; }

 private:
  Adafruit_MPU6050 mpu_;
  sensors_event_t accelEvent_{};
  sensors_event_t gyroEvent_{};
  sensors_event_t tempEvent_{};
  ImuSample sample_{};
  float gyroOffsetXDps_ = 0.0f;
  float gyroOffsetYDps_ = 0.0f;
  float gyroOffsetZDps_ = 0.0f;
  bool online_ = false;
  uint32_t lastUpdateMs_ = 0;
};
