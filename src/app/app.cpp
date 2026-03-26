#include "app/app.h"

#include <Arduino.h>

#include "bsp/board_pins.h"
#include "common/math_utils.h"

namespace {

constexpr uint32_t kImuPeriodUs = 2000;
constexpr uint32_t kControlPeriodUs = 5000;
constexpr uint32_t kLedPeriodMs = 50;
constexpr uint32_t kAssistPeriodMs = 500;

uint32_t ledColor(const Adafruit_NeoPixel& strip, const bool armed, const FaultCode fault,
                  const TuneMode mode) {
  if (fault != FaultCode::None) {
    return strip.Color(255, 0, 0);
  }
  if (!armed) {
    return strip.Color(32, 16, 0);
  }
  if (mode == TuneMode::AssistTune) {
    return strip.Color(0, 32, 64);
  }
  if (mode == TuneMode::ManualTune) {
    return strip.Color(0, 48, 16);
  }
  return strip.Color(0, 16, 48);
}

void applyMotorFlags(const uint8_t flags, MotorFeedback& motor) {
  motor.enabled = (flags & 0x01) != 0;
  motor.reached = (flags & 0x02) != 0;
  motor.blocked = (flags & 0x04) != 0;
  motor.blockedProtection = (flags & 0x08) != 0;
}

}  // namespace

void App::begin() {
  Serial.begin(BoardPins::kDebugBaudRate);
  delay(200);
  tuneProtocol_.begin(Serial);

  statusLed_.begin();
  statusLed_.setBrightness(48);
  statusLed_.show();

  pidStore_.begin();

  if (!pidStore_.loadFactory(factoryTuning_)) {
    factoryTuning_ = defaultControlTuning();
    pidStore_.saveFactory(factoryTuning_);
  }

  if (!pidStore_.loadActive(activeTuning_)) {
    activeTuning_ = factoryTuning_;
    pidStore_.saveActive(activeTuning_);
  }

  applyTuning(activeTuning_, true);

  imuReady_ = imu_.begin(wire_, BoardPins::kI2cSdaPin, BoardPins::kI2cSclPin);
  if (imuReady_) {
    imuReady_ = imu_.calibrateGyro();
  }

  if (!imuReady_) {
    enterFault(FaultCode::ImuOffline);
  }

  motorBus_.begin(Serial1, BoardPins::kMotorBaudRate, BoardPins::kMotorSerialRxPin,
                  BoardPins::kMotorSerialTxPin);
  stopMotors();

  telemetry_.tsMs = millis();
  telemetry_.tuneMode = tuneMode_;
  telemetry_.faultCode = safetyManager_.faultCode();
}

void App::update() {
  ParsedCommand command;
  while (tuneProtocol_.pollCommand(command)) {
    handleCommand(command);
  }

  pollImuTask();
  controlTask();
  motorPollTask();
  telemetryTask();
  ledTask();
}

void App::handleCommand(const ParsedCommand& command) {
  switch (command.type) {
    case ParsedCommand::Type::GetPid:
      tuneProtocol_.sendPid(activeTuning_);
      return;

    case ParsedCommand::Type::SetPid: {
      ControlTuning updated = activeTuning_;
      PIDGains* gains = gainsForGroup(command.group);
      if (gains == nullptr) {
        tuneProtocol_.sendError("invalid_group");
        return;
      }

      if (strcmp(command.group, "angle") == 0) {
        gains = &updated.angle;
      } else if (strcmp(command.group, "speed") == 0) {
        gains = &updated.speed;
      } else if (strcmp(command.group, "turn") == 0) {
        gains = &updated.turn;
      }

      if (command.hasKp) {
        gains->kp = command.kp;
      }
      if (command.hasKi) {
        gains->ki = command.ki;
      }
      if (command.hasKd) {
        gains->kd = command.kd;
      }
      if (command.hasIntegralLimit) {
        gains->integralLimit = command.integralLimit;
      }
      if (command.hasOutputLimit) {
        gains->outputLimit = command.outputLimit;
      }

      pendingConfig_.hasValue = true;
      pendingConfig_.tuning = updated;
      tuneMode_ = TuneMode::ManualTune;
      tuneProtocol_.sendAck("set_pid", true, "pending");
      return;
    }

    case ParsedCommand::Type::SavePid:
      if (pidStore_.saveActive(activeTuning_) && pidStore_.saveFactory(activeTuning_)) {
        factoryTuning_ = activeTuning_;
        tuneProtocol_.sendAck("save_pid", true);
      } else {
        tuneProtocol_.sendAck("save_pid", false, "nvs_write_failed");
      }
      return;

    case ParsedCommand::Type::RollbackPid:
      pendingConfig_.hasValue = false;
      activeTuning_ = factoryTuning_;
      applyTuning(activeTuning_, true);
      pidStore_.saveActive(activeTuning_);
      tuneProtocol_.sendAck("rollback_pid", true);
      return;

    case ParsedCommand::Type::SetMode:
      tuneMode_ = command.mode;
      tuneProtocol_.sendAck("set_mode", true);
      return;

    case ParsedCommand::Type::SetDrive:
      if (command.hasArm) {
        armed_ = command.arm;
        if (!armed_) {
          stopMotors();
        } else {
          motorBus_.enableMotor(BoardPins::kLeftMotorAddress, true);
          motorBus_.enableMotor(BoardPins::kRightMotorAddress, true);
        }
      }
      if (command.hasSpeedRpm) {
        userSpeedTargetRpm_ = command.speedRpm;
      }
      if (command.hasTurnRpm) {
        userTurnTargetRpm_ = command.turnRpm;
      }
      tuneProtocol_.sendAck("set_drive", true);
      return;

    case ParsedCommand::Type::GetStatus:
      tuneProtocol_.sendStatus(telemetry_, activeTuning_);
      return;

    case ParsedCommand::Type::SetLimit: {
      ControlTuning updated = activeTuning_;
      if (command.hasPitchTarget) {
        updated.pitchTargetDeg = command.pitchTarget;
      }
      if (command.hasRpmLimit) {
        updated.rpmLimit = command.rpmLimit;
      }
      if (command.hasTurnLimit) {
        updated.turnLimit = command.turnLimit;
      }
      if (command.hasMotorAccel) {
        updated.motorAccel = command.motorAccel;
      }
      if (command.hasTelemetryMs) {
        updated.telemetryPeriodMs = command.telemetryMs;
      }
      pendingConfig_.hasValue = true;
      pendingConfig_.tuning = updated;
      tuneProtocol_.sendAck("set_limit", true, "pending");
      return;
    }

    case ParsedCommand::Type::ClearFault:
      if (absValue(telemetry_.pitchDeg) < 10.0f) {
        safetyManager_.clear();
        tuneMode_ = TuneMode::Idle;
        tuneProtocol_.sendAck("clear_fault", true);
      } else {
        tuneProtocol_.sendAck("clear_fault", false, "pitch_not_safe");
      }
      return;

    case ParsedCommand::Type::None:
      return;
  }
}

void App::applyTuning(const ControlTuning& tuning, const bool resetControllers) {
  activeTuning_ = tuning;
  balanceController_.setGains(activeTuning_.angle);
  speedController_.setGains(activeTuning_.speed);

  if (resetControllers) {
    balanceController_.reset();
    speedController_.reset();
  }
}

bool App::canApplyPendingConfig() const {
  return absValue(telemetry_.pitchDeg) < 10.0f && safetyManager_.faultCode() == FaultCode::None;
}

void App::pollImuTask() {
  const uint32_t nowUs = micros();
  if (nowUs - lastImuTaskUs_ < kImuPeriodUs) {
    return;
  }
  lastImuTaskUs_ = nowUs;

  if (!imu_.update()) {
    enterFault(FaultCode::ImuOffline);
    return;
  }

  const ImuSample& sample = imu_.sample();
  const float dtSec = telemetry_.tsMs == 0 ? 0.002f : static_cast<float>(kImuPeriodUs) / 1.0e6f;
  telemetry_.pitchDeg = complementaryFilter_.update(sample.pitchAccelDeg, sample.gyroXDps, dtSec);
  telemetry_.gyroDegPerSec = sample.gyroXDps;
}

void App::controlTask() {
  const uint32_t nowUs = micros();
  if (nowUs - lastControlTaskUs_ < kControlPeriodUs) {
    return;
  }
  lastControlTaskUs_ = nowUs;

  telemetry_.tsMs = millis();
  telemetry_.tuneMode = tuneMode_;
  telemetry_.armed = armed_;

  if (pendingConfig_.hasValue && canApplyPendingConfig()) {
    applyTuning(pendingConfig_.tuning, false);
    pidStore_.saveActive(activeTuning_);
    pendingConfig_.hasValue = false;
    tuneProtocol_.sendPid(activeTuning_);
  }

  const float dtSec = static_cast<float>(kControlPeriodUs) / 1.0e6f;
  telemetry_.pitchTargetDeg = activeTuning_.pitchTargetDeg;

  const float averageMeasuredRpm =
      0.5f * (static_cast<float>(leftMotor_.actualRpm) + static_cast<float>(rightMotor_.actualRpm));
  filteredAverageRpm_ = lerpValue(filteredAverageRpm_, averageMeasuredRpm, 0.25f);

  telemetry_.speedOutput = speedController_.update(userSpeedTargetRpm_, filteredAverageRpm_, dtSec);
  telemetry_.turnOutput = clampValue(activeTuning_.turn.kp * userTurnTargetRpm_,
                                     -activeTuning_.turn.outputLimit, activeTuning_.turn.outputLimit);
  telemetry_.balanceOutput = balanceController_.update(
      telemetry_.pitchDeg, activeTuning_.pitchTargetDeg, telemetry_.gyroDegPerSec, dtSec);

  float baseRpm = telemetry_.balanceOutput + telemetry_.speedOutput;
  float leftTarget = baseRpm - telemetry_.turnOutput;
  float rightTarget = baseRpm + telemetry_.turnOutput;

  telemetry_.outputSaturated = false;
  if (absValue(leftTarget) > activeTuning_.rpmLimit || absValue(rightTarget) > activeTuning_.rpmLimit) {
    telemetry_.outputSaturated = true;
    telemetry_.metrics.outputSaturationCount++;
  }

  leftTarget = clampValue(leftTarget, -activeTuning_.rpmLimit, activeTuning_.rpmLimit);
  rightTarget = clampValue(rightTarget, -activeTuning_.rpmLimit, activeTuning_.rpmLimit);

  telemetry_.leftRpmTarget = leftTarget;
  telemetry_.rightRpmTarget = rightTarget;
  telemetry_.leftRpmActual = leftMotor_.actualRpm;
  telemetry_.rightRpmActual = rightMotor_.actualRpm;
  telemetry_.motorStateLeft = leftMotor_.stateFlags;
  telemetry_.motorStateRight = rightMotor_.stateFlags;

  telemetry_.metrics.angleAbsIntegral += absValue(telemetry_.pitchDeg) * dtSec;
  telemetry_.metrics.anglePeakDeg =
      max(telemetry_.metrics.anglePeakDeg, absValue(telemetry_.pitchDeg));
  telemetry_.metrics.rpmRipple =
      0.5f * (absValue(telemetry_.leftRpmTarget - telemetry_.leftRpmActual) +
              absValue(telemetry_.rightRpmTarget - telemetry_.rightRpmActual));

  telemetry_.faultCode = safetyManager_.evaluate(telemetry_, leftMotor_, rightMotor_, imu_.lastUpdateMs());

  if (telemetry_.faultCode != FaultCode::None) {
    enterFault(telemetry_.faultCode);
    return;
  }

  if (!armed_) {
    stopMotors();
    return;
  }

  motorBus_.velocityControl(
      BoardPins::kLeftMotorAddress,
      static_cast<int16_t>(leftTarget * BoardPins::kLeftMotorSign),
      activeTuning_.motorAccel);
  motorBus_.velocityControl(
      BoardPins::kRightMotorAddress,
      static_cast<int16_t>(rightTarget * BoardPins::kRightMotorSign),
      activeTuning_.motorAccel);

  if (tuneMode_ == TuneMode::AssistTune && telemetry_.tsMs - lastAssistMs_ >= kAssistPeriodMs) {
    lastAssistMs_ = telemetry_.tsMs;
    updateAssistSuggestion();
  }
}

void App::motorPollTask() {
  const uint32_t nowMs = millis();
  if (nowMs - lastMotorPollMs_ < activeTuning_.motorPollPeriodMs) {
    return;
  }
  lastMotorPollMs_ = nowMs;

  int16_t rpm = 0;
  uint8_t flags = 0;

  if (motorBus_.readRealTimeSpeed(BoardPins::kLeftMotorAddress, rpm)) {
    leftMotor_.online = true;
    leftMotor_.actualRpm = static_cast<int16_t>(rpm * BoardPins::kLeftMotorSign);
    leftMotor_.lastResponseMs = nowMs;
  }
  if (motorBus_.readStateFlag(BoardPins::kLeftMotorAddress, flags)) {
    leftMotor_.online = true;
    leftMotor_.stateFlags = flags;
    leftMotor_.lastResponseMs = nowMs;
    applyMotorFlags(flags, leftMotor_);
  }

  if (motorBus_.readRealTimeSpeed(BoardPins::kRightMotorAddress, rpm)) {
    rightMotor_.online = true;
    rightMotor_.actualRpm = static_cast<int16_t>(rpm * BoardPins::kRightMotorSign);
    rightMotor_.lastResponseMs = nowMs;
  }
  if (motorBus_.readStateFlag(BoardPins::kRightMotorAddress, flags)) {
    rightMotor_.online = true;
    rightMotor_.stateFlags = flags;
    rightMotor_.lastResponseMs = nowMs;
    applyMotorFlags(flags, rightMotor_);
  }
}

void App::telemetryTask() {
  const uint32_t nowMs = millis();
  if (nowMs - lastTelemetryMs_ < activeTuning_.telemetryPeriodMs) {
    return;
  }
  lastTelemetryMs_ = nowMs;
  tuneProtocol_.sendTelemetry(telemetry_);
}

void App::ledTask() {
  const uint32_t nowMs = millis();
  if (nowMs - lastLedMs_ < kLedPeriodMs) {
    return;
  }
  lastLedMs_ = nowMs;
  statusLed_.setPixelColor(0, ledColor(statusLed_, armed_, safetyManager_.faultCode(), tuneMode_));
  statusLed_.show();
}

void App::enterFault(const FaultCode code) {
  if (code != FaultCode::None) {
    telemetry_.faultCode = code;
    telemetry_.metrics.fallEventCount++;
    tuneMode_ = TuneMode::FaultLock;
  }
  armed_ = false;
  stopMotors();
}

void App::stopMotors() {
  motorBus_.stopNow(BoardPins::kLeftMotorAddress);
  motorBus_.stopNow(BoardPins::kRightMotorAddress);
  motorBus_.enableMotor(BoardPins::kLeftMotorAddress, false);
  motorBus_.enableMotor(BoardPins::kRightMotorAddress, false);
}

void App::updateAssistSuggestion() {
  const float pitchAbs = absValue(telemetry_.pitchDeg);
  if (pitchAbs > activeTuning_.assistPitchThresholdDeg &&
      telemetry_.metrics.rpmRipple < activeTuning_.rpmLimit * 0.25f) {
    tuneProtocol_.sendAssistSuggestion("increase_angle_kp_or_kd");
  } else if (telemetry_.metrics.rpmRipple > activeTuning_.rpmLimit * 0.45f) {
    tuneProtocol_.sendAssistSuggestion("reduce_angle_kp_or_increase_motor_limit");
  } else if (pitchAbs < 2.0f && absValue(userSpeedTargetRpm_) > 20.0f &&
             absValue(telemetry_.speedOutput) > activeTuning_.speed.outputLimit * 0.7f) {
    tuneProtocol_.sendAssistSuggestion("increase_speed_kp_or_ki");
  } else {
    tuneProtocol_.sendAssistSuggestion("state_stable");
  }
}

PIDGains* App::gainsForGroup(const char* group) {
  if (strcmp(group, "angle") == 0) {
    return &activeTuning_.angle;
  }
  if (strcmp(group, "speed") == 0) {
    return &activeTuning_.speed;
  }
  if (strcmp(group, "turn") == 0) {
    return &activeTuning_.turn;
  }
  return nullptr;
}
