#include "safety/safety_manager.h"

#include <Arduino.h>

#include "bsp/board_pins.h"
#include "common/math_utils.h"

FaultCode SafetyManager::evaluate(const TelemetrySnapshot& snapshot, const MotorFeedback& leftMotor,
                                  const MotorFeedback& rightMotor, const uint32_t imuLastUpdateMs) {
  FaultCode nextFault = FaultCode::None;

  if (snapshot.tsMs - imuLastUpdateMs > BoardPins::kImuTimeoutMs) {
    nextFault = FaultCode::ImuDataTimeout;
  } else if (absValue(snapshot.pitchDeg) > BoardPins::kPitchFaultLimitDeg) {
    nextFault = FaultCode::PitchLimitExceeded;
  } else if (snapshot.armed && (!leftMotor.online || snapshot.tsMs - leftMotor.lastResponseMs > BoardPins::kMotorTimeoutMs)) {
    nextFault = FaultCode::MotorLeftOffline;
  } else if (snapshot.armed &&
             (!rightMotor.online || snapshot.tsMs - rightMotor.lastResponseMs > BoardPins::kMotorTimeoutMs)) {
    nextFault = FaultCode::MotorRightOffline;
  } else if (leftMotor.blockedProtection || rightMotor.blockedProtection || leftMotor.blocked ||
             rightMotor.blocked) {
    nextFault = FaultCode::MotorFault;
  }

  if (nextFault != FaultCode::None) {
    latchedFault_ = nextFault;
  }

  return latchedFault_;
}

void SafetyManager::clear() { latchedFault_ = FaultCode::None; }
