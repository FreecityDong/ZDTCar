#pragma once

#include <Arduino.h>

#include "common/app_types.h"

class EmmV5Bus {
 public:
  bool begin(HardwareSerial& serial, uint32_t baudRate, int rxPin, int txPin);

  bool enableMotor(uint8_t address, bool enabled, bool syncFlag = false);
  bool velocityControl(uint8_t address, int16_t rpm, uint8_t accel, bool syncFlag = false);
  bool stopNow(uint8_t address, bool syncFlag = false);
  bool readRealTimeSpeed(uint8_t address, int16_t& rpm);
  bool readStateFlag(uint8_t address, uint8_t& flags);

 private:
  bool writeCommand(const uint8_t* data, size_t length);
  bool waitForFrame(uint8_t expectedAddress, uint8_t expectedFunction, uint8_t* buffer,
                    size_t& length, uint32_t timeoutMs = 4);

  HardwareSerial* serial_ = nullptr;
};
