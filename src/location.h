#pragma once
// ===========================================================================
//  location.h  -  observer location (manual lat/lon or Maidenhead grid)
// ===========================================================================
//  The StickSat cut-down has no GPS (CardSat's optional Grove / Cap-LoRa GNSS
//  is removed). Location is set over the web setup page as a grid square or as
//  decimal lat/lon, and persisted in Settings.
#include <Arduino.h>

struct Observer {
  double lat = 0.0;     // degrees +N
  double lon = 0.0;     // degrees +E
  double altM = 0.0;    // metres
  bool   valid = false;
};

class Location {
public:
  void setManual(double lat, double lon, double altM);
  bool setFromGrid(const String& grid);    // Maidenhead -> lat/lon (centre)

  const Observer& obs() const { return _obs; }

  static String toGrid(double lat, double lon);  // 6-char Maidenhead
  // Maidenhead -> lat/lon (square centre) without mutating any Location.
  static bool gridToLatLon(const String& grid, double& lat, double& lon);

private:
  Observer _obs;
};
