#pragma once
// ===========================================================================
//  settings.h  -  persistent user configuration (LittleFS JSON)
// ===========================================================================
//  Trimmed from CardSat: no radio model / CI-V address / CAT baud, no rotator,
//  no GPS source, no per-satellite calibration. Just what the cut-down needs.
#include <Arduino.h>
#include "config.h"

struct Settings {
  // WiFi (set by the captive portal on first boot)
  char     ssid[33] = "";
  char     pass[65] = "";
  // Orbital data source (GP/OMM JSON). Defaults to AMSAT.
  char     gpUrl[160] = AMSAT_GP_URL;
  // Location (set on the web setup page; grid or lat/lon)
  double   lat = 0.0, lon = 0.0, altM = 0.0;
  // Tracking
  float    minPassEl = 0.0f;   // include all passes (peak elevation >= 0 deg)
  bool     aosAlarm  = true;   // beep + flash before a favorite's AOS
  // True once the user has completed location + satellite selection at least
  // once; until then the device shows the "run setup" prompt.
  bool     setupDone = false;

  bool load();
  bool save();
};
