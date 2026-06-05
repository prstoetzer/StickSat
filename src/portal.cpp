// ===========================================================================
//  portal.cpp  -  captive WiFi portal + on-device setup web server
// ===========================================================================
#include "portal.h"
#include "config.h"
#include "settings.h"
#include "satdb.h"
#include "net.h"
#include "location.h"
#include "favs.h"
#include "buttons.h"
#include <M5Unified.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

namespace Portal {

static const char* AP_SSID = "StickSat-Setup";
static const byte   DNS_PORT = 53;

// --- tiny on-screen helper (mirrors app colours; kept local to the portal) --
static void screen(const String& l1, const String& l2 = "",
                   const String& l3 = "", uint16_t c = TFT_WHITE) {
  auto& d = M5.Display;
  d.fillScreen(TFT_BLACK);
  d.setTextSize(1);
  d.setTextColor(TFT_CYAN, TFT_BLACK);
  d.setCursor(4, 4);  d.print("StickSat setup");
  d.setTextColor(c, TFT_BLACK);
  d.setCursor(4, 24); d.print(l1);
  d.setTextColor(TFT_WHITE, TFT_BLACK);
  d.setCursor(4, 40); d.print(l2);
  d.setCursor(4, 56); d.print(l3);
}

static String htmlEscape(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); ++i) {
    char c = s[i];
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;";  break;
      case '>': o += "&gt;";  break;
      case '"': o += "&quot;"; break;
      default:  o += c;
    }
  }
  return o;
}

static String pageHead(const String& title) {
  return String(F(
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>")) + title + F("</title><style>"
    "body{font-family:system-ui,sans-serif;margin:0;background:#0b0f1a;color:#e8edf5}"
    ".w{max-width:560px;margin:0 auto;padding:18px}"
    "h1{font-size:20px;margin:6px 0 14px}"
    "label{display:block;margin:10px 0 4px;font-size:14px;color:#9fb0c8}"
    "input[type=text],input[type=password]{width:100%;padding:10px;border-radius:8px;"
    "border:1px solid #2a3550;background:#121a2b;color:#fff;box-sizing:border-box}"
    "button{margin-top:16px;width:100%;padding:12px;border:0;border-radius:8px;"
    "background:#2f6df6;color:#fff;font-size:16px}"
    ".sat{display:flex;align-items:center;gap:8px;padding:6px 4px;border-bottom:1px solid #1b2438}"
    ".sat input{width:18px;height:18px}"
    ".n{flex:1}.id{color:#7c8aa5;font-size:12px}"
    ".bar{position:sticky;top:0;background:#0b0f1a;padding:8px 0;z-index:2}"
    ".cnt{color:#9fb0c8;font-size:14px}"
    "</style></head><body><div class=w>");
}
static const char* PAGE_TAIL = "</div></body></html>";

// ===========================================================================
//  Phase 1: captive WiFi portal
// ===========================================================================
bool runWifiPortal(Settings& cfg, Net& net) {
  WebServer server(80);
  DNSServer dns;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  IPAddress apIP = WiFi.softAPIP();
  dns.start(DNS_PORT, "*", apIP);          // redirect every host to us

  bool   done = false;
  String pendErr;

  auto handleRoot = [&]() {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_FAILED) WiFi.scanNetworks(true);   // async
    String body = pageHead("WiFi setup");
    body += F("<h1>Connect StickSat to WiFi</h1>");
    if (pendErr.length()) {
      body += "<p style='color:#ff8080'>" + htmlEscape(pendErr) + "</p>";
      pendErr = "";
    }
    body += F("<form method=POST action=/save>");
    body += F("<label>Network (SSID)</label><input type=text name=ssid list=aps autocomplete=off>");
    body += F("<datalist id=aps>");
    if (n > 0) {
      // de-dup by name, strongest kept (scan list is RSSI-ordered already)
      for (int i = 0; i < n && i < 30; ++i) {
        String s = WiFi.SSID(i);
        if (s.length()) body += "<option value='" + htmlEscape(s) + "'>";
      }
    }
    body += F("</datalist>");
    body += F("<label>Password</label><input type=password name=pass autocomplete=off>");
    body += F("<button type=submit>Connect</button></form>");
    body += F("<p class=cnt>Pick or type your 2.4 GHz network. "
              "The list refreshes each time you reload.</p>");
    body += PAGE_TAIL;
    server.send(200, "text/html", body);
    if (n <= 0) WiFi.scanNetworks(true);   // kick a fresh async scan
  };

  auto handleSave = [&]() {
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    if (ssid.length() == 0) { pendErr = "SSID required"; server.sendHeader("Location","/"); server.send(302); return; }

    screen("Connecting to:", ssid, "...", TFT_YELLOW);
    // Stop SoftAP services for a clean STA association attempt.
    dns.stop();
    WiFi.softAPdisconnect(true);
    bool ok = net.connect(ssid, pass, 15000);
    if (ok) {
      strncpy(cfg.ssid, ssid.c_str(), sizeof(cfg.ssid) - 1); cfg.ssid[sizeof(cfg.ssid)-1]=0;
      strncpy(cfg.pass, pass.c_str(), sizeof(cfg.pass) - 1); cfg.pass[sizeof(cfg.pass)-1]=0;
      cfg.save();
      // We may not be able to deliver this page (AP is down), but try anyway.
      String body = pageHead("Connected");
      body += F("<h1>Connected!</h1><p>You can close this page. "
                "Continue setup on the device screen.</p>");
      body += PAGE_TAIL;
      server.send(200, "text/html", body);
      done = true;
    } else {
      // Bring the AP back so the user can retry.
      WiFi.mode(WIFI_AP);
      WiFi.softAP(AP_SSID);
      dns.start(DNS_PORT, "*", WiFi.softAPIP());
      pendErr = "Could not connect to " + ssid + " (" + net.lastErr + ")";
      server.sendHeader("Location", "/");
      server.send(302);
    }
  };

  // Captive-portal probe URLs -> redirect to root so the OS pops the page.
  auto redirectRoot = [&]() { server.sendHeader("Location", String("http://") + apIP.toString()); server.send(302); };

  server.on("/", handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/generate_204", redirectRoot);          // Android
  server.on("/hotspot-detect.html", redirectRoot);   // iOS / macOS
  server.on("/ncsi.txt", redirectRoot);              // Windows
  server.onNotFound(redirectRoot);
  server.begin();
  WiFi.scanNetworks(true);

  screen("Join WiFi:", String(AP_SSID), "then open 192.168.4.1", TFT_GREEN);

  uint32_t lastBlink = 0; bool on = true;
  while (!done) {
    dns.processNextRequest();
    server.handleClient();
    M5.update();
    // gentle "waiting" blink on the title so it's clearly alive
    if (millis() - lastBlink > 600) {
      lastBlink = millis(); on = !on;
      M5.Display.fillRect(4, 4, 120, 10, TFT_BLACK);
      M5.Display.setTextColor(on ? TFT_CYAN : TFT_BLUE, TFT_BLACK);
      M5.Display.setCursor(4, 4); M5.Display.print("StickSat setup");
    }
    delay(2);
  }

  server.stop();
  dns.stop();
  return true;
}

// ===========================================================================
//  Phase 2: location + satellite picker
// ===========================================================================
bool runSetupServer(Settings& cfg, SatDb& db, Net& net, Location& loc) {
  WebServer server(80);
  IPAddress ip = WiFi.localIP();

  // Working copy of the current favorites so the page reflects prior choices.
  static uint32_t fav[MAX_FAVS];
  int favN = Favs::load(fav, MAX_FAVS);

  bool finished = false;
  String notice;

  auto handleRoot = [&]() {
    String body = pageHead("StickSat setup");
    body += F("<h1>Location &amp; satellites</h1>");
    if (notice.length()) { body += "<p style='color:#7CFC7C'>" + htmlEscape(notice) + "</p>"; notice=""; }

    // --- location form ---
    body += F("<form method=POST action=/loc>");
    body += F("<label>Maidenhead grid (e.g. FN31pr) &mdash; OR fill lat/lon below</label>");
    body += F("<input type=text name=grid placeholder='grid square' value='");
    if (cfg.lat || cfg.lon) body += htmlEscape(Location::toGrid(cfg.lat, cfg.lon));
    body += F("'>");
    body += F("<label>Latitude (deg, +N)</label><input type=text name=lat value='");
    if (cfg.lat) body += String(cfg.lat, 5);
    body += F("'>");
    body += F("<label>Longitude (deg, +E)</label><input type=text name=lon value='");
    if (cfg.lon) body += String(cfg.lon, 5);
    body += F("'>");
    body += F("<label>Altitude (m, optional)</label><input type=text name=alt value='");
    if (cfg.altM) body += String(cfg.altM, 0);
    body += F("'>");
    body += F("<label>Minimum pass elevation (deg) &mdash; 0 = include passes "
              "down to the horizon</label><input type=text name=minel value='");
    body += String(cfg.minPassEl, 0);
    body += F("'>");
    body += F("<button type=submit>Save location &amp; settings</button></form>");

    // --- satellite picker ---
    body += F("<h1 style='margin-top:24px'>Satellites (max ");
    body += String(MAX_FAVS); body += F(")</h1>");
    if (db.count() == 0) {
      body += F("<p style='color:#ffd27a'>No GP data downloaded yet. Use "
                "<a style='color:#7ab' href=/update>Download AMSAT GP</a> first.</p>");
    } else {
      body += F("<form method=POST action=/sats>");
      body += F("<div class=bar><span class=cnt id=c>0</span> selected (max ");
      body += String(MAX_FAVS);
      body += F(")</div>");
      for (int i = 0; i < db.count(); ++i) {
        SatEntry& s = db.at(i);
        bool chk = Favs::contains(fav, favN, s.norad);
        body += F("<label class=sat><input type=checkbox name=s value=");
        body += String((unsigned long)s.norad);
        if (chk) body += F(" checked");
        body += F("><span class=n>");
        body += htmlEscape(String(s.name));
        body += F("</span><span class=id>");
        body += String((unsigned long)s.norad);
        body += F("</span></label>");
      }
      // Save button at the BOTTOM of the list, just above Finish setup.
      body += F("<button type=submit>Save satellite selection</button></form>");
      // client-side cap + live counter
      body += F("<script>"
        "var max=");
      body += String(MAX_FAVS);
      body += F(";function upd(){var b=document.querySelectorAll('input[name=s]');"
        "var n=0;b.forEach(function(x){if(x.checked)n++});"
        "document.getElementById('c').textContent=n;"
        "b.forEach(function(x){if(!x.checked)x.disabled=(n>=max)});}"
        "document.addEventListener('change',function(e){if(e.target.name=='s')upd()});"
        "upd();</script>");
    }

    body += F("<form method=POST action=/finish><button "
              "style='background:#1f9d55'>Finish setup</button></form>");
    body += F("<form method=POST action=/update><button "
              "style='background:#444'>Re-download AMSAT GP</button></form>");
    body += PAGE_TAIL;
    server.send(200, "text/html", body);
  };

  auto handleLoc = [&]() {
    String grid = server.arg("grid"); grid.trim();
    double lat = 0, lon = 0, alt = 0;
    bool haveLL = false;
    if (server.arg("lat").length() && server.arg("lon").length()) {
      lat = server.arg("lat").toDouble();
      lon = server.arg("lon").toDouble();
      alt = server.arg("alt").toDouble();
      haveLL = (lat != 0.0 || lon != 0.0);
    }
    if (grid.length() >= 4 && !haveLL) {
      double gl, go;
      if (Location::gridToLatLon(grid, gl, go)) { lat = gl; lon = go; haveLL = true; }
    }

    // Minimum pass elevation (clamped to a sensible 0..89 deg). Saved whether or
    // not the location field was valid, so it can be adjusted on its own.
    String notice2;
    if (server.arg("minel").length()) {
      float me = server.arg("minel").toFloat();
      if (me < 0)   me = 0;
      if (me > 89)  me = 89;
      cfg.minPassEl = me;
      notice2 = "  Min elevation: " + String(me, 0) + " deg";
    }

    if (haveLL) {
      cfg.lat = lat; cfg.lon = lon; cfg.altM = alt;
      loc.setManual(lat, lon, alt);
      notice = "Location saved: " + Location::toGrid(lat, lon) + notice2;
    } else if (notice2.length()) {
      notice = "Saved." + notice2;
    } else {
      notice = "Enter a valid grid square or lat/lon.";
    }
    cfg.save();
    server.sendHeader("Location", "/"); server.send(302);
  };

  auto handleSats = [&]() {
    int n = 0;
    for (int i = 0; i < server.args() && n < MAX_FAVS; ++i) {
      if (server.argName(i) == "s") {
        uint32_t id = (uint32_t)strtoul(server.arg(i).c_str(), nullptr, 10);
        if (id && !Favs::contains(fav, n, id)) fav[n++] = id;
      }
    }
    favN = n;
    Favs::save(fav, favN);
    notice = String(favN) + " satellite(s) saved.";
    server.sendHeader("Location", "/"); server.send(302);
  };

  auto handleUpdate = [&]() {
    screen("Downloading", "AMSAT GP data...", "(this takes a moment)", TFT_YELLOW);
    bool ok = net.fetchGpToFile(cfg.gpUrl, FILE_GP);
    if (ok) { db.loadGpFromFs(); notice = "GP downloaded: " + String(db.count()) + " sats."; }
    else      notice = "GP download failed: " + net.lastErr;
    screen("Setup server at:", ip.toString(), notice, ok ? TFT_GREEN : TFT_RED);
    server.sendHeader("Location", "/"); server.send(302);
  };

  auto handleFinish = [&]() {
    // Tell the browser we're done first, then cache transponders on-device
    // (the download can take a few seconds per satellite, longer than the
    // browser would wait, so we don't block the HTTP response on it).
    String body = pageHead("Done");
    body += F("<h1>Setup complete</h1><p>StickSat will now download transponder "
              "data for your satellites so it keeps working out of WiFi range, "
              "then start tracking. Watch the device screen for progress. "
              "You can close this page.</p>");
    body += PAGE_TAIL;
    server.send(200, "text/html", body);
    finished = true;
  };

  server.on("/", handleRoot);
  server.on("/loc",    HTTP_POST, handleLoc);
  server.on("/sats",   HTTP_POST, handleSats);
  server.on("/update", HTTP_POST, handleUpdate);
  server.on("/update", HTTP_GET,  handleUpdate);
  server.on("/finish", HTTP_POST, handleFinish);
  server.begin();

  screen("Setup server at:", ip.toString(), "Open it in a browser", TFT_GREEN);
  M5.Display.setCursor(4, 78);
  M5.Display.setTextColor(TFT_DARKGREY, TFT_BLACK);
  M5.Display.print("(hold KEY1 to skip)");

  while (!finished) {
    server.handleClient();
    M5.update();
    if (Keys::key1Held()) { finished = true; }  // hold KEY1 to bail out
    delay(2);
  }
  server.stop();

  // ---- Cache transponders for every selected satellite (offline use) -------
  //  Pull each satellite's SatNOGS transmitter list to flash now, while WiFi is
  //  still up, so the Track screen's Doppler/transponder data works later with
  //  no network. Re-load the favorites in case they were just changed.
  favN = Favs::load(fav, MAX_FAVS);
  if (favN && net.connected()) {
    for (int i = 0; i < favN; ++i) {
      int idx = db.indexOfNorad(fav[i]);
      const char* nm = (idx >= 0) ? db.at(idx).name : "";
      char line2[40];
      snprintf(line2, sizeof(line2), "%.20s (%lu)", nm, (unsigned long)fav[i]);
      screen("Caching transponders", line2,
             String(i + 1) + " / " + String(favN), TFT_YELLOW);
      String j;
      if (net.fetchSatnogsTransmitters(fav[i], j)) {
        SatDb::saveTxCache(fav[i], j);          // persist raw JSON for offline parse
      }
      delay(150);                               // be gentle on the SatNOGS API
    }
    screen("Transponders cached", "for " + String(favN) + " satellites",
           "Starting tracker...", TFT_GREEN);
    delay(1200);
  }

  cfg.setupDone = true; cfg.save();
  return true;
}

} // namespace Portal
