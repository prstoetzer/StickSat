// ===========================================================================
//  gps.cpp  -  minimal NMEA 0183 parser for the M5Stack Unit GPS v1.1
// ===========================================================================
#include "gps.h"
#include "config.h"

// UART2 on the Grove port. RX=33 (reads the unit's TX), TX=32 (to the unit's RX).
static HardwareSerial GpsSerial(2);

void Gps::begin() {
  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  _len = 0;
  Serial.printf("[gps] UART2 @%d  RX=G%d TX=G%d (optional)\n",
                GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN);
}

void Gps::update() {
  // Drain whatever is available; assemble complete NMEA lines and parse them.
  while (GpsSerial.available()) {
    char c = (char)GpsSerial.read();
    _lastByteMs = millis();
    if (c == '\r') continue;
    if (c == '\n') {
      _buf[_len] = 0;
      if (_len > 6) parseLine(_buf);
      _len = 0;
    } else if (_len < sizeof(_buf) - 1) {
      _buf[_len++] = c;
    } else {
      _len = 0;                 // overrun -> resync on next line
    }
  }
}

// Verify the optional "*hh" NMEA checksum if present; tolerate its absence.
static bool nmeaChecksumOk(const char* s) {
  if (s[0] != '$') return false;
  const char* star = strchr(s, '*');
  if (!star) return true;                       // no checksum field -> accept
  uint8_t sum = 0;
  for (const char* p = s + 1; p < star; ++p) sum ^= (uint8_t)*p;
  uint8_t want = (uint8_t)strtol(star + 1, nullptr, 16);
  return sum == want;
}

void Gps::parseLine(char* s) {
  if (!nmeaChecksumOk(s)) return;
  // Talker id varies (GP/GN/GL/...); match the sentence type after 3 chars.
  // e.g. "$GPRMC", "$GNRMC", "$GPGGA", "$GNGGA".
  if (strlen(s) < 6) return;
  const char* type = s + 3;                     // skip "$Gx"
  if (!strncmp(type, "RMC", 3)) parseRmc(s);
  else if (!strncmp(type, "GGA", 3)) parseGga(s);
}

// Split helper: returns pointer to field n (0-based) within the comma-delimited
// sentence, modifying the buffer in place (NUL-terminating fields).
static char* field(char* s, int n) {
  int i = 0;
  char* p = s;
  while (*p && i < n) { if (*p == ',') i++; p++; if (i == n) break; }
  if (i != n) return nullptr;
  // terminate this field at the next comma or '*'
  char* e = p;
  while (*e && *e != ',' && *e != '*') e++;
  *e = 0;
  return p;
}

// $GxRMC,hhmmss.ss,A,llll.lll,N,yyyyy.yyy,E,spd,crs,ddmmyy,...
void Gps::parseRmc(char* s) {
  // Work on a copy so the in-place field splitting doesn't clobber later parses.
  char tmp[100]; strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;

  char* tField = field(tmp, 1);
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* status = field(tmp, 2);
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* latF = field(tmp, 3);
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* ns = field(tmp, 4);
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* lonF = field(tmp, 5);
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* ew = field(tmp, 6);
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* dField = field(tmp, 9);

  bool active = (status && status[0] == 'A');

  // Time hhmmss(.sss) + date ddmmyy -> unix UTC.
  int hh = 0, mi = 0, sec = 0;
  if (tField && strlen(tField) >= 6) {
    hh  = (tField[0]-'0')*10 + (tField[1]-'0');
    mi  = (tField[2]-'0')*10 + (tField[3]-'0');
    sec = (tField[4]-'0')*10 + (tField[5]-'0');
  }
  if (dField && strlen(dField) >= 6) {
    _dd = (dField[0]-'0')*10 + (dField[1]-'0');
    _mo = (dField[2]-'0')*10 + (dField[3]-'0');
    _yy = 2000 + (dField[4]-'0')*10 + (dField[5]-'0');
  }
  if (_yy >= 2000 && tField && strlen(tField) >= 6) {
    // Civil UTC -> unix (Howard Hinnant), no timegm dependency.
    int Y = _yy, Mo = _mo, D = _dd;
    int yy = Y - (Mo <= 2);
    long era = (yy >= 0 ? yy : yy - 399) / 400;
    unsigned yoe = (unsigned)(yy - era * 400);
    unsigned doy = (153 * (Mo + (Mo > 2 ? -3 : 9)) + 2) / 5 + D - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    long days = era * 146097 + (long)doe - 719468;
    _utc = (time_t)days * 86400 + hh * 3600 + mi * 60 + sec;
    _timeValid = true;
  }

  if (active && latF && lonF && latF[0] && lonF[0]) {
    // ddmm.mmmm -> degrees.
    double rawLat = atof(latF), rawLon = atof(lonF);
    int latDeg = (int)(rawLat / 100);
    int lonDeg = (int)(rawLon / 100);
    double lat = latDeg + (rawLat - latDeg * 100) / 60.0;
    double lon = lonDeg + (rawLon - lonDeg * 100) / 60.0;
    if (ns && ns[0] == 'S') lat = -lat;
    if (ew && ew[0] == 'W') lon = -lon;
    _lat = lat; _lon = lon;
    _hasFix = true;
  } else if (!active) {
    _hasFix = false;
  }
}

// $GxGGA,hhmmss.ss,llll.lll,N,yyyyy.yyy,E,fix,sats,hdop,alt,M,...
void Gps::parseGga(char* s) {
  char tmp[100];
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* fixF = field(tmp, 6);
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* satF = field(tmp, 7);
  strncpy(tmp, s, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
  char* altF = field(tmp, 9);

  if (satF && satF[0]) _sats = atoi(satF);
  if (altF && altF[0]) _altM = atof(altF);
  if (fixF && fixF[0]) { int q = atoi(fixF); if (q >= 1) _hasFix = true; }
}
