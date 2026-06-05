// ===========================================================================
//  net.cpp  -  WiFi + HTTPS (reused from CardSat, scan helper removed)
// ===========================================================================
#include "net.h"
#include "config.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include "storage.h"
#include <time.h>

bool Net::connect(const String& ssid, const String& pass, uint32_t timeoutMs) {
  if (ssid.length() == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) delay(150);
  return WiFi.status() == WL_CONNECTED;
}

bool Net::connected() { return WiFi.status() == WL_CONNECTED; }

void Net::syncTimeNtp() {
  // UTC (no offset, no DST). Pool servers.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  struct tm ti;
  for (int i = 0; i < 40 && !getLocalTime(&ti, 250); ++i) { /* wait */ }
}

bool Net::httpsGet(const String& url, String& out, size_t maxBytes) {
  lastCode = 0; lastErr = "";
  if (!connected()) { lastErr = "no WiFi"; return false; }

  Serial.printf("[net] GET %s\n", url.c_str());

  WiFiClientSecure client;
  client.setInsecure();           // public data; pin a CA root for production
  client.setTimeout(15000);

  HTTPClient http;
  http.setUserAgent("StickSat-StickS3/1.0");
  http.setConnectTimeout(15000);
  http.setTimeout(15000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.useHTTP10(true);

  if (!http.begin(client, url)) { lastErr = "begin failed"; return false; }
  http.addHeader("Accept", "*/*");

  int code = http.GET();
  lastCode = code;
  if (code != HTTP_CODE_OK) {
    lastErr = (code > 0) ? ("HTTP " + String(code)) : HTTPClient::errorToString(code);
    Serial.printf("[net] GET failed: %d (%s)\n", code, lastErr.c_str());
    http.end();
    return false;
  }

  int len = http.getSize();
  out = "";
  if (len > 0) out.reserve(min((size_t)len + 16, maxBytes));

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[512];
  size_t total = 0;
  uint32_t lastRx = millis();
  while (total < maxBytes) {
    size_t avail = stream->available();
    if (avail) {
      int r = stream->readBytes(buf, min(avail, sizeof(buf)));
      if (r <= 0) break;
      out.concat((const char*)buf, r);
      total += r;
      lastRx = millis();
      if (len > 0 && total >= (size_t)len) break;
    } else {
      if (len > 0 && total >= (size_t)len) break;
      if (!http.connected() && !stream->available() && millis() - lastRx > 500) break;
      if (millis() - lastRx > 10000) break;
      delay(5);
    }
  }
  http.end();
  if (out.length() == 0) { lastErr = "empty body"; return false; }
  return true;
}

bool Net::httpsGetToFile(const String& url, const char* path,
                         size_t maxBytes, size_t* written) {
  lastCode = 0; lastErr = "";
  if (written) *written = 0;
  if (!connected()) { lastErr = "no WiFi"; return false; }

  Serial.printf("[net] GET %s -> %s\n", url.c_str(), path);

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15000);

  HTTPClient http;
  http.setUserAgent("StickSat-StickS3/1.0");
  http.setConnectTimeout(15000);
  http.setTimeout(15000);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.useHTTP10(true);

  if (!http.begin(client, url)) { lastErr = "begin failed"; return false; }
  http.addHeader("Accept", "*/*");

  int code = http.GET();
  lastCode = code;
  if (code != HTTP_CODE_OK) {
    lastErr = (code > 0) ? ("HTTP " + String(code)) : HTTPClient::errorToString(code);
    Serial.printf("[net] GET failed: %d (%s)\n", code, lastErr.c_str());
    http.end();
    return false;
  }

  File f = Store::fs().open(path, "w");
  if (!f) { lastErr = "fs open failed"; http.end(); return false; }

  int len = http.getSize();
  WiFiClient* stream = http.getStreamPtr();
  uint8_t buf[512];
  size_t total = 0;
  uint32_t lastRx = millis();
  bool writeErr = false;
  // Stream straight to flash so no large contiguous RAM buffer is ever needed.
  while (total < maxBytes) {
    size_t avail = stream->available();
    if (avail) {
      int r = stream->readBytes(buf, min(avail, sizeof(buf)));
      if (r <= 0) break;
      if (f.write(buf, r) != (size_t)r) { writeErr = true; break; }
      total += r;
      lastRx = millis();
      if (len > 0 && total >= (size_t)len) break;
    } else {
      if (len > 0 && total >= (size_t)len) break;
      if (!http.connected() && !stream->available() && millis() - lastRx > 500) break;
      if (millis() - lastRx > 10000) break;
      delay(5);
    }
  }
  f.close();
  http.end();
  if (written) *written = total;
  Serial.printf("[net] streamed %u bytes to %s (declared %d)\n",
                (unsigned)total, path, len);
  if (writeErr)   { lastErr = "fs write failed"; return false; }
  if (total == 0) { lastErr = "empty body"; return false; }
  return true;
}

bool Net::fetchGpToFile(const String& url, const char* path) {
  return httpsGetToFile(url, path, 400000, nullptr);
}

bool Net::fetchSatnogsTransmitters(uint32_t norad, String& out) {
  String url = String(SATNOGS_TX_URL) + String((unsigned long)norad);
  return httpsGet(url, out, 60000);
}
