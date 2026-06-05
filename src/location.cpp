// ===========================================================================
//  location.cpp
// ===========================================================================
#include "location.h"
#include <math.h>

void Location::setManual(double lat, double lon, double altM) {
  _obs.lat = lat; _obs.lon = lon; _obs.altM = altM;
  _obs.valid = true;
}

// Maidenhead grid -> lat/lon (centre of the square). Accepts 4 or 6 chars.
bool Location::gridToLatLon(const String& gridIn, double& latOut, double& lonOut) {
  String g = gridIn; g.trim(); g.toUpperCase();
  if (g.length() < 4) return false;
  if (g[0] < 'A' || g[0] > 'R' || g[1] < 'A' || g[1] > 'R') return false;
  if (g[2] < '0' || g[2] > '9' || g[3] < '0' || g[3] > '9') return false;
  double lon = (g[0] - 'A') * 20.0 - 180.0;
  double lat = (g[1] - 'A') * 10.0 - 90.0;
  lon += (g[2] - '0') * 2.0;
  lat += (g[3] - '0') * 1.0;
  if (g.length() >= 6) {
    lon += (g[4] - 'A') * (2.0 / 24.0) + (1.0 / 24.0);
    lat += (g[5] - 'A') * (1.0 / 24.0) + (0.5 / 24.0);
  } else {
    lon += 1.0; lat += 0.5;   // centre of the 2x1 deg square
  }
  latOut = lat; lonOut = lon;
  return true;
}

bool Location::setFromGrid(const String& gridIn) {
  double lat, lon;
  if (!gridToLatLon(gridIn, lat, lon)) return false;
  setManual(lat, lon, 0.0);
  return true;
}

String Location::toGrid(double lat, double lon) {
  lon += 180.0; lat += 90.0;
  char g[7];
  g[0] = 'A' + (int)(lon / 20.0);
  g[1] = 'A' + (int)(lat / 10.0);
  g[2] = '0' + (int)(fmod(lon, 20.0) / 2.0);
  g[3] = '0' + (int)(fmod(lat, 10.0) / 1.0);
  g[4] = 'A' + (int)(fmod(lon, 2.0) / (2.0 / 24.0));
  g[5] = 'A' + (int)(fmod(lat, 1.0) / (1.0 / 24.0));
  g[6] = 0;
  return String(g);
}
