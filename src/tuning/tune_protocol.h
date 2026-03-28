#pragma once

#include <Arduino.h>

#include "common/app_types.h"

struct ParsedCommand {
  enum class Type : uint8_t {
    None,
    GetPid,
    SetPid,
    SavePid,
    RollbackPid,
    SetMode,
    SetDrive,
    MotorTest,
    GetStatus,
    SetLimit,
    ClearFault,
  };

  Type type = Type::None;
  char group[12] = {};
  TuneMode mode = TuneMode::Idle;
  bool hasKp = false;
  bool hasKi = false;
  bool hasKd = false;
  bool hasIntegralLimit = false;
  bool hasOutputLimit = false;
  bool hasPitchTarget = false;
  bool hasRpmLimit = false;
  bool hasTurnLimit = false;
  bool hasMotorAccel = false;
  bool hasTelemetryMs = false;
  bool hasArm = false;
  bool hasSpeedRpm = false;
  bool hasTurnRpm = false;
  bool hasMotorTestRpm = false;
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float integralLimit = 0.0f;
  float outputLimit = 0.0f;
  float pitchTarget = 0.0f;
  float rpmLimit = 0.0f;
  float turnLimit = 0.0f;
  uint8_t motorAccel = 0;
  uint16_t telemetryMs = 0;
  bool arm = false;
  float speedRpm = 0.0f;
  float turnRpm = 0.0f;
  char motorTarget[8] = {};
  float motorTestRpm = 0.0f;
};

class TuneProtocol {
 public:
  void begin(Stream& stream);
  bool pollCommand(ParsedCommand& command);
  void sendAck(const char* cmd, bool ok, const char* message = nullptr);
  void sendError(const char* message);
  void sendPid(const ControlTuning& tuning);
  void sendStatus(const TelemetrySnapshot& snapshot, const ControlTuning& tuning);
  void sendTelemetry(const TelemetrySnapshot& snapshot);
  void sendAssistSuggestion(const char* suggestion);

 private:
  bool parseLine(const char* line, ParsedCommand& command);

  Stream* stream_ = nullptr;
  char lineBuffer_[256] = {};
  size_t lineLength_ = 0;
};
