// ===========================================================================
//  settings.cpp
// ===========================================================================
#include "settings.h"
#include "config.h"
#include <LittleFS.h>
#include "storage.h"
#include <ArduinoJson.h>

bool Settings::load() {
  File f = Store::fs().open(FILE_CFG, "r");
  if (!f) return false;
  JsonDocument d;
  if (deserializeJson(d, f)) { f.close(); return false; }
  f.close();

  strncpy(ssid, d["ssid"] | "", sizeof(ssid) - 1);
  strncpy(pass, d["pass"] | "", sizeof(pass) - 1);
  strncpy(gpUrl, d["gpurl"] | AMSAT_GP_URL, sizeof(gpUrl) - 1);
  gpUrl[sizeof(gpUrl) - 1] = 0;
  lat        = d["lat"] | 0.0;
  lon        = d["lon"] | 0.0;
  altM       = d["alt"] | 0.0;
  minPassEl  = d["minel"] | 0.0f;
  aosAlarm   = d["aosalarm"] | true;
  setupDone  = d["setupdone"] | false;
  return true;
}

bool Settings::save() {
  JsonDocument d;
  d["ssid"]      = ssid;
  d["pass"]      = pass;
  d["gpurl"]     = gpUrl;
  d["lat"]       = lat;
  d["lon"]       = lon;
  d["alt"]       = altM;
  d["minel"]     = minPassEl;
  d["aosalarm"]  = aosAlarm;
  d["setupdone"] = setupDone;
  File f = Store::fs().open(FILE_CFG, "w");
  if (!f) return false;
  serializeJson(d, f);
  f.close();
  return true;
}
