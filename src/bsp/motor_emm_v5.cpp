#include "bsp/motor_emm_v5.h"

#include <string.h>

bool EmmV5Bus::begin(HardwareSerial& serial, const uint32_t baudRate, const int rxPin,
                     const int txPin) {
  serial_ = &serial;
  serial_->begin(baudRate, SERIAL_8N1, rxPin, txPin);
  return true;
}

bool EmmV5Bus::enableMotor(const uint8_t address, const bool enabled, const bool syncFlag) {
  const uint8_t command[] = {address, 0xF3, 0xAB, static_cast<uint8_t>(enabled ? 1 : 0),
                             static_cast<uint8_t>(syncFlag ? 1 : 0), 0x6B};
  return writeCommand(command, sizeof(command));
}

bool EmmV5Bus::velocityControl(const uint8_t address, const int16_t rpm, const uint8_t accel,
                               const bool syncFlag) {
  const uint16_t velocity = static_cast<uint16_t>(abs(rpm));
  const uint8_t direction = rpm < 0 ? 1 : 0;
  const uint8_t command[] = {address,
                             0xF6,
                             direction,
                             static_cast<uint8_t>(velocity >> 8),
                             static_cast<uint8_t>(velocity & 0xFF),
                             accel,
                             static_cast<uint8_t>(syncFlag ? 1 : 0),
                             0x6B};
  return writeCommand(command, sizeof(command));
}

bool EmmV5Bus::stopNow(const uint8_t address, const bool syncFlag) {
  const uint8_t command[] = {address, 0xFE, 0x98, static_cast<uint8_t>(syncFlag ? 1 : 0), 0x6B};
  return writeCommand(command, sizeof(command));
}

bool EmmV5Bus::readRealTimeSpeed(const uint8_t address, int16_t& rpm) {
  const uint8_t command[] = {address, 0x35, 0x6B};
  if (!writeCommand(command, sizeof(command))) {
    return false;
  }

  uint8_t frame[8] = {};
  size_t length = 0;
  if (!waitForFrame(address, 0x35, frame, length) || length < 5) {
    return false;
  }

  const int16_t raw = static_cast<int16_t>((frame[3] << 8) | frame[4]);
  rpm = frame[2] == 0x01 ? -raw : raw;
  return true;
}

bool EmmV5Bus::readStateFlag(const uint8_t address, uint8_t& flags) {
  const uint8_t command[] = {address, 0x3A, 0x6B};
  if (!writeCommand(command, sizeof(command))) {
    return false;
  }

  uint8_t frame[6] = {};
  size_t length = 0;
  if (!waitForFrame(address, 0x3A, frame, length) || length < 4) {
    return false;
  }

  flags = frame[2];
  return true;
}

bool EmmV5Bus::writeCommand(const uint8_t* data, const size_t length) {
  if (serial_ == nullptr) {
    return false;
  }

  while (serial_->available() > 0) {
    serial_->read();
  }

  return serial_->write(data, length) == static_cast<size_t>(length);
}

bool EmmV5Bus::waitForFrame(const uint8_t expectedAddress, const uint8_t expectedFunction,
                            uint8_t* buffer, size_t& length, const uint32_t timeoutMs) {
  if (serial_ == nullptr) {
    return false;
  }

  length = 0;
  uint32_t lastByteMs = millis();
  const uint32_t startMs = lastByteMs;

  while (millis() - startMs <= timeoutMs) {
    while (serial_->available() > 0 && length < 32) {
      buffer[length++] = serial_->read();
      lastByteMs = millis();
    }

    if (length >= 4 && millis() - lastByteMs >= 1) {
      break;
    }

    delay(0);
  }

  if (length < 4) {
    return false;
  }

  if (buffer[0] != expectedAddress || buffer[1] != expectedFunction || buffer[length - 1] != 0x6B) {
    return false;
  }

  return true;
}
