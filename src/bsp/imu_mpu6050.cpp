#include "bsp/imu_mpu6050.h"

#include <math.h>

#include "bsp/board_pins.h"

bool ImuMpu6050::begin(TwoWire& wire, const int sdaPin, const int sclPin) {
  wire.begin(sdaPin, sclPin, 400000U);

  if (!mpu_.begin(0x68, &wire)) {
    online_ = false;
    return false;
  }

  mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_.setFilterBandwidth(MPU6050_BAND_44_HZ);
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
    if (!mpu_.getEvent(&accelEvent_, &gyroEvent_, &tempEvent_)) {
      online_ = false;
      return false;
    }

    sumX += gyroEvent_.gyro.x * RAD_TO_DEG;
    sumY += gyroEvent_.gyro.y * RAD_TO_DEG;
    sumZ += gyroEvent_.gyro.z * RAD_TO_DEG;
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

  if (!mpu_.getEvent(&accelEvent_, &gyroEvent_, &tempEvent_)) {
    online_ = false;
    return false;
  }

  sample_.valid = true;
  sample_.accelXMps2 = accelEvent_.acceleration.x;
  sample_.accelYMps2 = accelEvent_.acceleration.y;
  sample_.accelZMps2 = accelEvent_.acceleration.z;
  sample_.gyroXDps = (gyroEvent_.gyro.x * RAD_TO_DEG - gyroOffsetXDps_) * BoardPins::kPitchGyroSign;
  sample_.gyroYDps = (gyroEvent_.gyro.y * RAD_TO_DEG - gyroOffsetYDps_);
  sample_.gyroZDps = (gyroEvent_.gyro.z * RAD_TO_DEG - gyroOffsetZDps_);
  sample_.pitchAccelDeg =
      atan2f(sample_.accelXMps2 * BoardPins::kPitchAccelSign, sample_.accelZMps2) * RAD_TO_DEG;
  sample_.sampleTimeUs = micros();
  lastUpdateMs_ = millis();
  return true;
}
