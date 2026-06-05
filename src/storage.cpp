// ===========================================================================
//  storage.cpp
// ===========================================================================
#include "storage.h"
#include <LittleFS.h>

namespace Store {

static bool g_ready = false;

bool begin() {
  g_ready = false;
  // begin(true) formats the LittleFS partition if it mounts dirty / on first
  // boot, then mounts it. Requires a SPIFFS/LittleFS data partition to exist
  // in the flashed partition table (the default 8 MB table provides one).
  if (LittleFS.begin(true)) {
    g_ready = true;
    Serial.printf("[fs] LittleFS mounted (%u/%u bytes used)\n",
                  (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
    return true;
  }
  Serial.println("[fs] LittleFS mount FAILED (no data partition?)");
  return false;
}

fs::FS& fs() { return LittleFS; }
bool ready() { return g_ready; }
bool format() { return LittleFS.format(); }

} // namespace Store
