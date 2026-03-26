#pragma once

#include <Preferences.h>

#include "common/app_types.h"

class PidStore {
 public:
  bool begin();
  bool loadActive(ControlTuning& tuning);
  bool loadFactory(ControlTuning& tuning);
  bool saveActive(const ControlTuning& tuning);
  bool saveFactory(const ControlTuning& tuning);

 private:
  static constexpr uint32_t kMagic = 0x5A445443;
  static constexpr uint16_t kVersion = 1;

  struct PersistedBlob {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    ControlTuning tuning;
  };

  bool loadBlob(const char* key, ControlTuning& tuning);
  bool saveBlob(const char* key, const ControlTuning& tuning);

  Preferences preferences_;
};
