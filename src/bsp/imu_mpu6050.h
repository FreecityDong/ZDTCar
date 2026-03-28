#pragma once

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
  bool probeAddress(uint8_t address);
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t startReg, uint8_t* buffer, size_t length);
  bool readRawSample(int16_t& accX, int16_t& accY, int16_t& accZ, int16_t& gyroX, int16_t& gyroY,
                     int16_t& gyroZ);

  TwoWire* wire_ = nullptr;
  uint8_t deviceAddress_ = 0;
  ImuSample sample_{};
  float gyroOffsetXDps_ = 0.0f;
  float gyroOffsetYDps_ = 0.0f;
  float gyroOffsetZDps_ = 0.0f;
  bool online_ = false;
  uint32_t lastUpdateMs_ = 0;
};
