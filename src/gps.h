#pragma once
// ===========================================================================
//  gps.h  -  M5Stack Unit GPS v1.1 (ATGM336H/AT6668) on the Grove port
// ===========================================================================
//  The StickC Plus Grove (HY2.0-4P, PORT.CUSTOM) maps: yellow=G32, white=G33.
//  The GPS unit maps: yellow=UART_RX, white=UART_TX. With a straight Grove
//  cable the Stick therefore RECEIVES GPS data on G33 (the unit's TX) and would
//  transmit to the unit on G32. So we open a UART with RX=33, TX=32 at the
//  unit's default 115200 8N1 and parse NMEA 0183.
//
//  This is optional hardware: if nothing is plugged in, no bytes arrive, no fix
//  is ever reported, and (per the requirement) nothing is shown to the user.
//  A self-contained minimal NMEA parser (RMC + GGA) avoids pulling in an extra
//  library and matches the streaming style used elsewhere.
#include <Arduino.h>

class Gps {
public:
  void begin();                 // start the UART on the Grove port
  void update();                // feed available bytes into the NMEA parser

  bool   hasFix() const  { return _hasFix; }
  bool   timeValid() const { return _timeValid; }   // RMC date+time parsed
  time_t utc() const     { return _utc; }           // last parsed UTC (unix s)
  double lat() const     { return _lat; }           // degrees +N
  double lon() const     { return _lon; }           // degrees +E
  double altM() const    { return _altM; }          // metres (GGA)
  int    sats() const    { return _sats; }          // satellites in use (GGA)
  uint32_t lastByteMs() const { return _lastByteMs; } // for "GPS present" hint

private:
  void parseLine(char* s);
  void parseRmc(char* s);
  void parseGga(char* s);

  char     _buf[100];
  uint8_t  _len = 0;
  bool     _hasFix = false;
  bool     _timeValid = false;
  time_t   _utc = 0;
  double   _lat = 0, _lon = 0, _altM = 0;
  int      _sats = 0;
  uint32_t _lastByteMs = 0;
  // most-recent date (from RMC) so a time-only sentence can still build a stamp
  int      _yy = 0, _mo = 0, _dd = 0;
};
