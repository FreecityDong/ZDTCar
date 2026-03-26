#pragma once

#include <Adafruit_NeoPixel.h>
#include <Wire.h>

#include "bsp/board_pins.h"
#include "bsp/imu_mpu6050.h"
#include "bsp/motor_emm_v5.h"
#include "common/app_types.h"
#include "control/balance_controller.h"
#include "control/complementary_filter.h"
#include "control/speed_controller.h"
#include "safety/safety_manager.h"
#include "tuning/pid_store.h"
#include "tuning/tune_protocol.h"

class App {
 public:
  void begin();
  void update();

 private:
  void handleCommand(const ParsedCommand& command);
  void applyTuning(const ControlTuning& tuning, bool resetControllers);
  bool canApplyPendingConfig() const;
  void pollImuTask();
  void controlTask();
  void motorPollTask();
  void telemetryTask();
  void ledTask();
  void enterFault(FaultCode code);
  void stopMotors();
  void updateAssistSuggestion();
  PIDGains* gainsForGroup(const char* group);

  TwoWire wire_ = TwoWire(0);
  ImuMpu6050 imu_;
  EmmV5Bus motorBus_;
  ComplementaryFilter complementaryFilter_;
  BalanceController balanceController_;
  SpeedController speedController_;
  SafetyManager safetyManager_;
  PidStore pidStore_;
  TuneProtocol tuneProtocol_;
  Adafruit_NeoPixel statusLed_{BoardPins::kNeoPixelCount, BoardPins::kNeoPixelPin,
                               NEO_GRB + NEO_KHZ800};

  ControlTuning activeTuning_ = defaultControlTuning();
  ControlTuning factoryTuning_ = defaultControlTuning();
  PendingConfig pendingConfig_{};
  TelemetrySnapshot telemetry_{};
  MotorFeedback leftMotor_{};
  MotorFeedback rightMotor_{};

  bool imuReady_ = false;
  bool armed_ = false;
  TuneMode tuneMode_ = TuneMode::Idle;
  float userSpeedTargetRpm_ = 0.0f;
  float userTurnTargetRpm_ = 0.0f;
  float filteredAverageRpm_ = 0.0f;
  uint32_t lastImuTaskUs_ = 0;
  uint32_t lastControlTaskUs_ = 0;
  uint32_t lastMotorPollMs_ = 0;
  uint32_t lastTelemetryMs_ = 0;
  uint32_t lastLedMs_ = 0;
  uint32_t lastAssistMs_ = 0;
};
