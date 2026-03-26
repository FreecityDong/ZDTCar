#pragma once

#include <Arduino.h>

namespace BoardPins {

constexpr uint8_t kNeoPixelPin = 48;
constexpr uint16_t kNeoPixelCount = 1;

constexpr int kI2cSdaPin = 8;
constexpr int kI2cSclPin = 9;
constexpr int kMpuIntPin = 4;

constexpr int kMotorSerialRxPin = 17;
constexpr int kMotorSerialTxPin = 18;

constexpr uint8_t kLeftMotorAddress = 1;
constexpr uint8_t kRightMotorAddress = 2;

constexpr float kPitchAccelSign = 1.0f;
constexpr float kPitchGyroSign = 1.0f;
constexpr float kLeftMotorSign = 1.0f;
constexpr float kRightMotorSign = -1.0f;

constexpr uint32_t kDebugBaudRate = 115200;
constexpr uint32_t kMotorBaudRate = 512000;

constexpr float kPitchFaultLimitDeg = 30.0f;
constexpr uint32_t kImuTimeoutMs = 100;
constexpr uint32_t kMotorTimeoutMs = 200;

}  // namespace BoardPins
