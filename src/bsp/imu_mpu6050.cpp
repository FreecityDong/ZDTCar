#include "bsp/imu_mpu6050.h"

#include <math.h>

#include "bsp/board_pins.h"

namespace {

constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegPowerMgmt1 = 0x6B;
constexpr uint8_t kRegAccelXOutH = 0x3B;
constexpr float kGravity = 9.80665f;
constexpr float kAccelLsbPerG = 16384.0f;   // +/-2g
constexpr float kGyroLsbPerDps = 131.0f;    // +/-250dps

}  // namespace

bool ImuMpu6050::begin(TwoWire& wire, const int sdaPin, const int sclPin) {
  wire_ = &wire;
  wire_->begin(sdaPin, sclPin, BoardPins::kI2cFrequencyHz);
  delay(10);

  if (probeAddress(BoardPins::kMpuPrimaryAddress)) {
    deviceAddress_ = BoardPins::kMpuPrimaryAddress;
  } else if (probeAddress(BoardPins::kMpuSecondaryAddress)) {
    deviceAddress_ = BoardPins::kMpuSecondaryAddress;
  } else {
    online_ = false;
    return false;
  }

  if (!writeRegister(kRegPowerMgmt1, 0x00)) {
    online_ = false;
    return false;
  }
  delay(10);

  // Match the standalone sketch assumptions: gyro +/-250dps, accel +/-2g.
  if (!writeRegister(kRegGyroConfig, 0x00) || !writeRegister(kRegAccelConfig, 0x00) ||
      !writeRegister(kRegConfig, 0x03)) {
    online_ = false;
    return false;
  }

  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  if (!readRawSample(accX, accY, accZ, gyroX, gyroY, gyroZ)) {
    online_ = false;
    return false;
  }

  lastUpdateMs_ = millis();
  online_ = true;
  return true;
}

bool ImuMpu6050::calibrateGyro(const size_t sampleCount) {
  if (!online_) {
    return false;
  }

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  for (size_t i = 0; i < sampleCount; ++i) {
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;
    if (!readRawSample(accX, accY, accZ, gyroX, gyroY, gyroZ)) {
      online_ = false;
      return false;
    }

    sumX += static_cast<float>(gyroX) / kGyroLsbPerDps;
    sumY += static_cast<float>(gyroY) / kGyroLsbPerDps;
    sumZ += static_cast<float>(gyroZ) / kGyroLsbPerDps;
    delay(2);
  }

  gyroOffsetXDps_ = sumX / sampleCount;
  gyroOffsetYDps_ = sumY / sampleCount;
  gyroOffsetZDps_ = sumZ / sampleCount;
  return true;
}

bool ImuMpu6050::update() {
  if (!online_) {
    return false;
  }

  int16_t accX = 0;
  int16_t accY = 0;
  int16_t accZ = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;
  if (!readRawSample(accX, accY, accZ, gyroX, gyroY, gyroZ)) {
    online_ = false;
    return false;
  }

  const float accelX = static_cast<float>(accX) * kGravity / kAccelLsbPerG;
  const float accelY = static_cast<float>(accY) * kGravity / kAccelLsbPerG;
  const float accelZ = static_cast<float>(accZ) * kGravity / kAccelLsbPerG;
  const float gyroXDps = static_cast<float>(gyroX) / kGyroLsbPerDps;
  const float gyroYDps = static_cast<float>(gyroY) / kGyroLsbPerDps;
  const float gyroZDps = static_cast<float>(gyroZ) / kGyroLsbPerDps;

  sample_.valid = true;
  sample_.accelXMps2 = accelX;
  sample_.accelYMps2 = accelY;
  sample_.accelZMps2 = accelZ;
  sample_.gyroXDps = (gyroXDps - gyroOffsetXDps_) * BoardPins::kPitchGyroSign;
  sample_.gyroYDps = (gyroYDps - gyroOffsetYDps_);
  sample_.gyroZDps = (gyroZDps - gyroOffsetZDps_);
  sample_.pitchAccelDeg =
      atan2f(sample_.accelXMps2 * BoardPins::kPitchAccelSign, sample_.accelZMps2) * RAD_TO_DEG;
  sample_.sampleTimeUs = micros();
  lastUpdateMs_ = millis();
  return true;
}

bool ImuMpu6050::probeAddress(const uint8_t address) {
  if (wire_ == nullptr) {
    return false;
  }
  wire_->beginTransmission(address);
  return wire_->endTransmission() == 0;
}

bool ImuMpu6050::writeRegister(const uint8_t reg, const uint8_t value) {
  if (wire_ == nullptr || deviceAddress_ == 0) {
    return false;
  }
  wire_->beginTransmission(deviceAddress_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool ImuMpu6050::readRegisters(const uint8_t startReg, uint8_t* buffer, const size_t length) {
  if (wire_ == nullptr || deviceAddress_ == 0 || buffer == nullptr || length == 0 ||
      length > 32) {
    return false;
  }

  wire_->beginTransmission(deviceAddress_);
  wire_->write(startReg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }

  const size_t received = wire_->requestFrom(static_cast<int>(deviceAddress_),
                                             static_cast<int>(length), static_cast<int>(true));
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    if (wire_->available() <= 0) {
      return false;
    }
    buffer[i] = static_cast<uint8_t>(wire_->read());
  }
  return true;
}

bool ImuMpu6050::readRawSample(int16_t& accX, int16_t& accY, int16_t& accZ, int16_t& gyroX,
                               int16_t& gyroY, int16_t& gyroZ) {
  uint8_t bytes[14] = {};
  if (!readRegisters(kRegAccelXOutH, bytes, sizeof(bytes))) {
    return false;
  }

  accX = static_cast<int16_t>((bytes[0] << 8) | bytes[1]);
  accY = static_cast<int16_t>((bytes[2] << 8) | bytes[3]);
  accZ = static_cast<int16_t>((bytes[4] << 8) | bytes[5]);
  gyroX = static_cast<int16_t>((bytes[8] << 8) | bytes[9]);
  gyroY = static_cast<int16_t>((bytes[10] << 8) | bytes[11]);
  gyroZ = static_cast<int16_t>((bytes[12] << 8) | bytes[13]);
  return true;
}
