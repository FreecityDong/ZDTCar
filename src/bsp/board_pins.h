#pragma once

#include <Arduino.h>

namespace BoardPins {

constexpr uint8_t kNeoPixelPin = 48;
constexpr uint16_t kNeoPixelCount = 1;

constexpr int kI2cSdaPin = 4;
constexpr int kI2cSclPin = 5;
constexpr int kMpuIntPin = 4;
constexpr uint32_t kI2cFrequencyHz = 100000;
constexpr uint8_t kMpuPrimaryAddress = 0x68;
constexpr uint8_t kMpuSecondaryAddress = 0x69;

constexpr int kLeftMotorSerialRxPin = 18;
constexpr int kLeftMotorSerialTxPin = 17;
constexpr int kRightMotorSerialRxPin = 13;
constexpr int kRightMotorSerialTxPin = 14;

constexpr uint8_t kLeftMotorAddress = 1;
constexpr uint8_t kRightMotorAddress = 2;

constexpr float kPitchAccelSign = 1.0f;
constexpr float kPitchGyroSign = 1.0f;
constexpr float kLeftMotorSign = 1.0f;
constexpr float kRightMotorSign = -1.0f;

constexpr uint32_t kDebugBaudRate = 115200;
constexpr uint32_t kMotorBaudRate = 115200;

constexpr float kPitchFaultLimitDeg = 30.0f;
constexpr uint32_t kImuTimeoutMs = 100;
constexpr uint32_t kMotorTimeoutMs = 200;

}  // namespace BoardPins
