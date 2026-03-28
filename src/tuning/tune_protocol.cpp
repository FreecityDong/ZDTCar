#include "tuning/tune_protocol.h"

#include <ArduinoJson.h>
#include <string.h>

namespace {

const char* tuneModeToString(const TuneMode mode) {
  switch (mode) {
    case TuneMode::Idle:
      return "idle";
    case TuneMode::ManualTune:
      return "manual_tune";
    case TuneMode::AssistTune:
      return "assist_tune";
    case TuneMode::AutoTunePrepare:
      return "auto_tune_prepare";
    case TuneMode::AutoTuneRunning:
      return "auto_tune_running";
    case TuneMode::AutoTuneEvaluate:
      return "auto_tune_evaluate";
    case TuneMode::AutoTuneApply:
      return "auto_tune_apply";
    case TuneMode::AutoTuneRollback:
      return "auto_tune_rollback";
    case TuneMode::FaultLock:
      return "fault_lock";
  }

  return "unknown";
}

const char* faultCodeToString(const FaultCode code) {
  switch (code) {
    case FaultCode::None:
      return "none";
    case FaultCode::ImuOffline:
      return "imu_offline";
    case FaultCode::ImuDataTimeout:
      return "imu_timeout";
    case FaultCode::PitchLimitExceeded:
      return "pitch_limit";
    case FaultCode::MotorLeftOffline:
      return "motor_left_offline";
    case FaultCode::MotorRightOffline:
      return "motor_right_offline";
    case FaultCode::MotorFault:
      return "motor_fault";
    case FaultCode::CommandNotArmed:
      return "not_armed";
  }

  return "unknown";
}

void fillPidObject(JsonObject object, const PIDGains& gains) {
  object["kp"] = gains.kp;
  object["ki"] = gains.ki;
  object["kd"] = gains.kd;
  object["integral_limit"] = gains.integralLimit;
  object["output_limit"] = gains.outputLimit;
}

TuneMode parseTuneMode(const char* mode) {
  if (strcmp(mode, "manual_tune") == 0) {
    return TuneMode::ManualTune;
  }
  if (strcmp(mode, "assist_tune") == 0) {
    return TuneMode::AssistTune;
  }
  if (strcmp(mode, "auto_tune_prepare") == 0) {
    return TuneMode::AutoTunePrepare;
  }
  if (strcmp(mode, "auto_tune_running") == 0) {
    return TuneMode::AutoTuneRunning;
  }
  if (strcmp(mode, "auto_tune_evaluate") == 0) {
    return TuneMode::AutoTuneEvaluate;
  }
  if (strcmp(mode, "auto_tune_apply") == 0) {
    return TuneMode::AutoTuneApply;
  }
  if (strcmp(mode, "auto_tune_rollback") == 0) {
    return TuneMode::AutoTuneRollback;
  }
  if (strcmp(mode, "fault_lock") == 0) {
    return TuneMode::FaultLock;
  }
  return TuneMode::Idle;
}

}  // namespace

void TuneProtocol::begin(Stream& stream) { stream_ = &stream; }

bool TuneProtocol::pollCommand(ParsedCommand& command) {
  if (stream_ == nullptr) {
    return false;
  }

  while (stream_->available() > 0) {
    const char ch = static_cast<char>(stream_->read());
    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      lineBuffer_[lineLength_] = '\0';
      const bool parsed = parseLine(lineBuffer_, command);
      lineLength_ = 0;
      return parsed;
    }

    if (lineLength_ + 1 < sizeof(lineBuffer_)) {
      lineBuffer_[lineLength_++] = ch;
    } else {
      lineLength_ = 0;
      sendError("command_too_long");
    }
  }

  return false;
}

void TuneProtocol::sendAck(const char* cmd, const bool ok, const char* message) {
  if (stream_ == nullptr) {
    return;
  }

  JsonDocument doc;
  doc["type"] = "ack";
  doc["cmd"] = cmd;
  doc["ok"] = ok;
  if (message != nullptr) {
    doc["message"] = message;
  }
  serializeJson(doc, *stream_);
  stream_->println();
}

void TuneProtocol::sendError(const char* message) {
  if (stream_ == nullptr) {
    return;
  }

  JsonDocument doc;
  doc["type"] = "error";
  doc["message"] = message;
  serializeJson(doc, *stream_);
  stream_->println();
}

void TuneProtocol::sendPid(const ControlTuning& tuning) {
  if (stream_ == nullptr) {
    return;
  }

  JsonDocument doc;
  doc["type"] = "pid";
  fillPidObject(doc["angle"].to<JsonObject>(), tuning.angle);
  fillPidObject(doc["speed"].to<JsonObject>(), tuning.speed);
  fillPidObject(doc["turn"].to<JsonObject>(), tuning.turn);
  doc["pitch_target"] = tuning.pitchTargetDeg;
  doc["rpm_limit"] = tuning.rpmLimit;
  doc["turn_limit"] = tuning.turnLimit;
  doc["motor_accel"] = tuning.motorAccel;
  doc["telemetry_ms"] = tuning.telemetryPeriodMs;
  serializeJson(doc, *stream_);
  stream_->println();
}

void TuneProtocol::sendStatus(const TelemetrySnapshot& snapshot, const ControlTuning& tuning) {
  if (stream_ == nullptr) {
    return;
  }

  JsonDocument doc;
  doc["type"] = "status";
  doc["armed"] = snapshot.armed;
  doc["mode"] = tuneModeToString(snapshot.tuneMode);
  doc["fault"] = faultCodeToString(snapshot.faultCode);
  doc["pitch"] = snapshot.pitchDeg;
  doc["gyro"] = snapshot.gyroDegPerSec;
  doc["pitch_target"] = tuning.pitchTargetDeg;
  doc["rpm_limit"] = tuning.rpmLimit;
  doc["motor_accel"] = tuning.motorAccel;
  serializeJson(doc, *stream_);
  stream_->println();
}

void TuneProtocol::sendTelemetry(const TelemetrySnapshot& snapshot) {
  if (stream_ == nullptr) {
    return;
  }

  JsonDocument doc;
  doc["type"] = "telemetry";
  doc["ts"] = snapshot.tsMs;
  doc["mode"] = tuneModeToString(snapshot.tuneMode);
  doc["armed"] = snapshot.armed;
  doc["fault"] = faultCodeToString(snapshot.faultCode);
  doc["pitch"] = snapshot.pitchDeg;
  doc["gyro"] = snapshot.gyroDegPerSec;
  doc["pitch_target"] = snapshot.pitchTargetDeg;
  doc["l_rpm_t"] = snapshot.leftRpmTarget;
  doc["r_rpm_t"] = snapshot.rightRpmTarget;
  doc["l_rpm"] = snapshot.leftRpmActual;
  doc["r_rpm"] = snapshot.rightRpmActual;
  doc["balance_out"] = snapshot.balanceOutput;
  doc["speed_out"] = snapshot.speedOutput;
  doc["turn_out"] = snapshot.turnOutput;
  doc["sat"] = snapshot.outputSaturated;
  doc["motor_left"] = snapshot.motorStateLeft;
  doc["motor_right"] = snapshot.motorStateRight;
  doc["angle_abs_integral"] = snapshot.metrics.angleAbsIntegral;
  doc["angle_peak"] = snapshot.metrics.anglePeakDeg;
  doc["rpm_ripple"] = snapshot.metrics.rpmRipple;
  doc["saturation_count"] = snapshot.metrics.outputSaturationCount;
  doc["fall_count"] = snapshot.metrics.fallEventCount;
  serializeJson(doc, *stream_);
  stream_->println();
}

void TuneProtocol::sendAssistSuggestion(const char* suggestion) {
  if (stream_ == nullptr) {
    return;
  }

  JsonDocument doc;
  doc["type"] = "assist";
  doc["suggestion"] = suggestion;
  serializeJson(doc, *stream_);
  stream_->println();
}

bool TuneProtocol::parseLine(const char* line, ParsedCommand& command) {
  command = ParsedCommand{};

  if (line == nullptr || line[0] == '\0') {
    return false;
  }

  JsonDocument doc;
  const DeserializationError error = deserializeJson(doc, line);
  if (error) {
    sendError("invalid_json");
    return false;
  }

  const char* cmd = doc["cmd"];
  if (cmd == nullptr) {
    sendError("missing_cmd");
    return false;
  }

  if (strcmp(cmd, "get_pid") == 0) {
    command.type = ParsedCommand::Type::GetPid;
    return true;
  }

  if (strcmp(cmd, "save_pid") == 0) {
    command.type = ParsedCommand::Type::SavePid;
    return true;
  }

  if (strcmp(cmd, "rollback_pid") == 0) {
    command.type = ParsedCommand::Type::RollbackPid;
    return true;
  }

  if (strcmp(cmd, "get_status") == 0) {
    command.type = ParsedCommand::Type::GetStatus;
    return true;
  }

  if (strcmp(cmd, "clear_fault") == 0) {
    command.type = ParsedCommand::Type::ClearFault;
    return true;
  }

  if (strcmp(cmd, "set_pid") == 0) {
    const char* group = doc["group"];
    if (group == nullptr) {
      sendError("missing_group");
      return false;
    }
    command.type = ParsedCommand::Type::SetPid;
    strlcpy(command.group, group, sizeof(command.group));
    command.hasKp = !doc["kp"].isNull();
    command.hasKi = !doc["ki"].isNull();
    command.hasKd = !doc["kd"].isNull();
    command.hasIntegralLimit = !doc["integral_limit"].isNull();
    command.hasOutputLimit = !doc["output_limit"].isNull();
    command.kp = doc["kp"] | 0.0f;
    command.ki = doc["ki"] | 0.0f;
    command.kd = doc["kd"] | 0.0f;
    command.integralLimit = doc["integral_limit"] | 0.0f;
    command.outputLimit = doc["output_limit"] | 0.0f;
    return true;
  }

  if (strcmp(cmd, "set_mode") == 0) {
    const char* mode = doc["mode"];
    if (mode == nullptr) {
      sendError("missing_mode");
      return false;
    }
    command.type = ParsedCommand::Type::SetMode;
    command.mode = parseTuneMode(mode);
    return true;
  }

  if (strcmp(cmd, "set_drive") == 0) {
    command.type = ParsedCommand::Type::SetDrive;
    command.hasArm = !doc["arm"].isNull();
    command.hasSpeedRpm = !doc["speed_rpm"].isNull();
    command.hasTurnRpm = !doc["turn_rpm"].isNull();
    command.arm = doc["arm"] | false;
    command.speedRpm = doc["speed_rpm"] | 0.0f;
    command.turnRpm = doc["turn_rpm"] | 0.0f;
    return true;
  }

  if (strcmp(cmd, "motor_test") == 0) {
    const char* target = doc["target"];
    if (target == nullptr) {
      sendError("missing_target");
      return false;
    }
    if (doc["rpm"].isNull()) {
      sendError("missing_rpm");
      return false;
    }

    command.type = ParsedCommand::Type::MotorTest;
    strlcpy(command.motorTarget, target, sizeof(command.motorTarget));
    command.hasMotorTestRpm = true;
    command.motorTestRpm = doc["rpm"] | 0.0f;
    return true;
  }

  if (strcmp(cmd, "set_limit") == 0) {
    command.type = ParsedCommand::Type::SetLimit;
    command.hasPitchTarget = !doc["pitch_target"].isNull();
    command.hasRpmLimit = !doc["rpm_limit"].isNull();
    command.hasTurnLimit = !doc["turn_limit"].isNull();
    command.hasMotorAccel = !doc["motor_accel"].isNull();
    command.hasTelemetryMs = !doc["telemetry_ms"].isNull();
    command.pitchTarget = doc["pitch_target"] | 0.0f;
    command.rpmLimit = doc["rpm_limit"] | 0.0f;
    command.turnLimit = doc["turn_limit"] | 0.0f;
    command.motorAccel = doc["motor_accel"] | 0;
    command.telemetryMs = doc["telemetry_ms"] | 0;
    return true;
  }

  sendError("unknown_cmd");
  return false;
}
