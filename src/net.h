#pragma once
// ===========================================================================
//  net.h  -  WiFi + HTTPS downloads (AMSAT GP, SatNOGS transponders)
// ===========================================================================
//  Reused from CardSat. The WiFi *scan* helper is dropped: on the StickS3 the
//  user picks the network from the captive-portal page (its own scan), not from
//  an on-device list.
#include <Arduino.h>

class Net {
public:
  bool connect(const String& ssid, const String& pass, uint32_t timeoutMs = 15000);
  bool connected();
  void syncTimeNtp();                       // sets system clock via NTP (UTC)

  // GET a URL over HTTPS into `out`. Returns false on HTTP/transport error.
  bool httpsGet(const String& url, String& out, size_t maxBytes = 200000);

  // GET a URL over HTTPS straight into a LittleFS file (no large RAM buffer).
  // The GP file (~75 KB+) is streamed to flash a chunk at a time so it never
  // needs a single large contiguous String on the heap.
  bool httpsGetToFile(const String& url, const char* path,
                      size_t maxBytes = 400000, size_t* written = nullptr);

  // Convenience wrappers.
  bool fetchGpToFile(const String& url, const char* path);   // GP -> cache file
  bool fetchSatnogsTransmitters(uint32_t norad, String& out);

  // Diagnostics from the most recent request.
  int    lastCode = 0;     // HTTP status (>0) or HTTPClient error (<0)
  String lastErr  = "";    // short human-readable reason
};
