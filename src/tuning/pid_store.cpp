#include "tuning/pid_store.h"

#include <string.h>

bool PidStore::begin() { return preferences_.begin("zdtcar", false); }

bool PidStore::loadActive(ControlTuning& tuning) { return loadBlob("active", tuning); }

bool PidStore::loadFactory(ControlTuning& tuning) { return loadBlob("factory", tuning); }

bool PidStore::saveActive(const ControlTuning& tuning) { return saveBlob("active", tuning); }

bool PidStore::saveFactory(const ControlTuning& tuning) { return saveBlob("factory", tuning); }

bool PidStore::loadBlob(const char* key, ControlTuning& tuning) {
  PersistedBlob blob{};
  if (preferences_.getBytesLength(key) != sizeof(blob)) {
    return false;
  }

  const size_t loaded = preferences_.getBytes(key, &blob, sizeof(blob));
  if (loaded != sizeof(blob) || blob.magic != kMagic || blob.version != kVersion) {
    return false;
  }

  tuning = blob.tuning;
  return true;
}

bool PidStore::saveBlob(const char* key, const ControlTuning& tuning) {
  PersistedBlob blob{};
  blob.magic = kMagic;
  blob.version = kVersion;
  blob.tuning = tuning;

  return preferences_.putBytes(key, &blob, sizeof(blob)) == sizeof(blob);
}
