#pragma once

#include <Arduino.h>

enum class TuneMode : uint8_t {
  Idle = 0,
  ManualTune = 1,
  AssistTune = 2,
  AutoTunePrepare = 3,
  AutoTuneRunning = 4,
  AutoTuneEvaluate = 5,
  AutoTuneApply = 6,
  AutoTuneRollback = 7,
  FaultLock = 8,
};

enum class FaultCode : uint8_t {
  None = 0,
  ImuOffline = 1,
  ImuDataTimeout = 2,
  PitchLimitExceeded = 3,
  MotorLeftOffline = 4,
  MotorRightOffline = 5,
  MotorFault = 6,
  CommandNotArmed = 7,
};

struct PIDGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float integralLimit = 0.0f;
  float outputLimit = 0.0f;
};

struct ControlTuning {
  PIDGains angle;
  PIDGains speed;
  PIDGains turn;
  float pitchTargetDeg = 0.0f;
  float rpmLimit = 400.0f;
  float turnLimit = 150.0f;
  uint8_t motorAccel = 0;
  uint16_t telemetryPeriodMs = 20;
  uint16_t motorPollPeriodMs = 20;
  float assistPitchThresholdDeg = 6.0f;
};

struct MotorFeedback {
  bool online = false;
  bool enabled = false;
  bool reached = false;
  bool blocked = false;
  bool blockedProtection = false;
  int16_t actualRpm = 0;
  uint8_t stateFlags = 0;
  uint32_t lastResponseMs = 0;
};

struct TelemetryMetrics {
  float angleAbsIntegral = 0.0f;
  float anglePeakDeg = 0.0f;
  float rpmRipple = 0.0f;
  uint32_t outputSaturationCount = 0;
  uint32_t fallEventCount = 0;
  uint32_t recoveryTimeMs = 0;
};

struct TelemetrySnapshot {
  uint32_t tsMs = 0;
  float pitchDeg = 0.0f;
  float gyroDegPerSec = 0.0f;
  float pitchTargetDeg = 0.0f;
  float leftRpmTarget = 0.0f;
  float rightRpmTarget = 0.0f;
  float leftRpmActual = 0.0f;
  float rightRpmActual = 0.0f;
  float balanceOutput = 0.0f;
  float speedOutput = 0.0f;
  float turnOutput = 0.0f;
  bool outputSaturated = false;
  uint8_t motorStateLeft = 0;
  uint8_t motorStateRight = 0;
  FaultCode faultCode = FaultCode::None;
  TuneMode tuneMode = TuneMode::Idle;
  bool armed = false;
  TelemetryMetrics metrics;
};

struct PendingConfig {
  bool hasValue = false;
  ControlTuning tuning;
};

inline ControlTuning defaultControlTuning() {
  ControlTuning tuning;
  tuning.angle = PIDGains{38.0f, 0.0f, 0.92f, 12.0f, 450.0f};
  tuning.speed = PIDGains{0.45f, 0.02f, 0.0f, 150.0f, 120.0f};
  tuning.turn = PIDGains{0.80f, 0.0f, 0.0f, 0.0f, 120.0f};
  tuning.pitchTargetDeg = 0.0f;
  tuning.rpmLimit = 450.0f;
  tuning.turnLimit = 140.0f;
  tuning.motorAccel = 0;
  tuning.telemetryPeriodMs = 20;
  tuning.motorPollPeriodMs = 20;
  tuning.assistPitchThresholdDeg = 6.0f;
  return tuning;
}
