#pragma once

#include "common/app_types.h"

class SafetyManager {
 public:
  FaultCode evaluate(const TelemetrySnapshot& snapshot, const MotorFeedback& leftMotor,
                     const MotorFeedback& rightMotor, uint32_t imuLastUpdateMs);
  void clear();
  FaultCode faultCode() const { return latchedFault_; }
  bool isFaulted() const { return latchedFault_ != FaultCode::None; }

 private:
  FaultCode latchedFault_ = FaultCode::None;
};
