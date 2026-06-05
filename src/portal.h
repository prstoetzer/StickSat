#pragma once
// ===========================================================================
//  portal.h  -  captive WiFi portal + on-device setup web server
// ===========================================================================
//  Two phases, both served by the same WebServer instance:
//
//  Phase 1 (no WiFi credentials yet)  -- runWifiPortal()
//      The device becomes a SoftAP ("StickSat-Setup") with a DNS server that
//      redirects everything to itself (captive portal). A page scans for
//      networks and collects SSID + password. On submit it tries to connect;
//      on success the credentials are saved and the function returns.
//
//  Phase 2 (connected to WiFi)        -- runSetupServer()
//      Hosts a setup site on the station IP where the user sets their location
//      (grid square or lat/lon) and picks up to MAX_FAVS satellites from the
//      downloaded AMSAT list. "Finish" marks setup complete and returns.
//
//  Both are blocking helpers driven from App; they pump M5.update() so the
//  on-screen instructions (and the KEY1 "skip/continue" affordance) stay live.
#include <Arduino.h>

class Settings;
class SatDb;
class Net;
class Location;

namespace Portal {
  // Run the captive portal until the user submits working WiFi credentials
  // (saved into cfg). Returns true once connected. `cfg` and `net` are used to
  // attempt the connection; the on-screen status is drawn via the callbacks.
  bool runWifiPortal(Settings& cfg, Net& net);

  // Run the location + satellite-picker site until the user taps Finish (or
  // KEY1 is held). Reads the satellite list from db and the current favorites
  // file; writes location into cfg and favorites to FILE_FAVS. Returns true
  // when the user finished (setup complete).
  bool runSetupServer(Settings& cfg, SatDb& db, Net& net, Location& loc);
}
