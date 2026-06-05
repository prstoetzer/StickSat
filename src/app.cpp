// ===========================================================================
//  app.cpp  -  UI state machine + rendering (StickSat)
// ===========================================================================
//  Adapted from CardSat's app.cpp. The Doppler/CAT service loop, rotator,
//  per-satellite calibration, mutual-window and manual-entry screens are all
//  removed. What remains: the Next-Passes schedule, the live polar plot, a
//  read-only Doppler/transponder readout, the AOS alarm and deep sleep.
//
//  Rendering uses M5Unified (M5.Display) with an off-screen M5Canvas sprite,
//  exactly as CardSat did on the Cardputer (same 135x240 panel, rotation 1 =>
//  240x135 landscape).
#include "app.h"
#include "config.h"
#include "favs.h"
#include "buttons.h"
#include "portal.h"
#include <M5Unified.h>
#include <WiFi.h>
#include <LittleFS.h>
#include "storage.h"
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <esp_sleep.h>

// 16-bit 565 colours (same palette CardSat used)
static const uint16_t CL_BLACK=0x0000, CL_WHITE=0xFFFF, CL_GREEN=0x07E0, CL_RED=0xF800,
                      CL_YELLOW=0xFFE0, CL_CYAN=0x07FF, CL_ORANGE=0xFD20, CL_GREY=0x7BEF,
                      CL_BLUE=0x041F, CL_DGREEN=0x0320;

// IMPORTANT: do NOT bind the canvas to &M5.Display here. This object is
// constructed during C++ static initialization, whose order relative to
// M5Unified's global `M5` object is undefined -- referencing M5.Display before
// M5.begin() has constructed it can fault at boot (watchdog reset loop). We use
// the no-argument constructor and set the target in App::setup() after M5.begin().
static M5Canvas canvas;

// Active drawing target: the off-screen sprite when it allocated (buffered,
// flicker-free), otherwise the panel itself (un-buffered fallback for the
// no-PSRAM PICO-D4). All draw calls go through `g`; only sprite creation and
// the final blit know which one is in use. Set in App::setup().
static LovyanGFX* g = nullptr;
static bool       g_buffered = false;

// Blit the sprite to the panel (no-op when drawing un-buffered). Push to an
// explicit destination so we don't depend on the sprite's parent being set.
static inline void flush() { if (g_buffered) canvas.pushSprite(&M5.Display, 0, 0); }

// --- small time helpers (verbatim from CardSat) ----------------------------
static bool timeIsSet() { time_t t = time(nullptr); return t > 1700000000; }

static String fmtHM(time_t t) {
  struct tm tmv; gmtime_r(&t, &tmv);
  char b[16]; snprintf(b, sizeof(b), "%02d:%02d", tmv.tm_hour, tmv.tm_min);
  return String(b);
}
static String fmtMDHM(time_t t) {
  struct tm tmv; gmtime_r(&t, &tmv);
  char b[20]; snprintf(b, sizeof(b), "%02d/%02d %02d:%02d",
                       tmv.tm_mon+1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min);
  return String(b);
}
static String fmtClock(time_t t) {
  struct tm tmv; gmtime_r(&t, &tmv);
  char b[16]; snprintf(b, sizeof(b), "%02d:%02d:%02d",
                       tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
  return String(b);
}
static String fmtMHz(uint32_t hz) {
  char b[20]; snprintf(b, sizeof(b), "%.5f", hz / 1e6); return String(b);
}
static String fmtCountdown(long s) {
  if (s < 0) s = 0;
  char b[12];
  if (s < 60)        snprintf(b, sizeof(b), "%lds", s);
  else if (s < 3600) snprintf(b, sizeof(b), "%ldm", s / 60);
  else               snprintf(b, sizeof(b), "%ldh%02ld", s / 3600, (s % 3600) / 60);
  return String(b);
}

// --- GP element-set age (staleness), verbatim from CardSat ------------------
static double gpAgeDays(const SatEntry& s) {
  if (!timeIsSet() || s.epochUnix <= 0) return -1;
  return ((double)time(nullptr) - s.epochUnix) / 86400.0;
}
static uint16_t ageColor(double d) {
  if (d < 0)  return CL_GREY;
  if (d < 14) return CL_GREEN;
  if (d < 28) return CL_YELLOW;
  return CL_RED;
}

// --- speaker beep (AOS alarm) ----------------------------------------------
static void beep(uint16_t freq, uint16_t ms) {
  M5.Speaker.tone(freq, ms);
}

// ===========================================================================
//  Setup
// ===========================================================================
void App::setup() {
  // M5Unified is begun in main.cpp before App::setup(); we just configure the
  // display/canvas here so the same code path works whether or not we ran the
  // setup portal first.
  M5.Display.setRotation(1);                 // 240x135 landscape
  M5.Speaker.setVolume(180);

  // Default drawing target = the panel itself, until/unless a sprite allocates.
  // (We do NOT bind the canvas to the display at static-init time -- see the
  // note on the `canvas` global above. flush() pushes to M5.Display explicitly,
  // so the sprite needs no parent.)
  g = &M5.Display; g_buffered = false;

  // Off-screen sprite for flicker-free drawing. The PICO-D4 has no PSRAM, so a
  // full 240x135x16bpp sprite (~64 KB) may not allocate once WiFi/TLS have
  // fragmented the heap. If it fails we keep drawing straight to the panel
  // (slightly more flicker, fully functional).
  canvas.setTextWrap(false);
  canvas.setPsram(false);
  canvas.setColorDepth(16);
  // Either the full 16bpp sprite allocates (buffered, correct RGB565 colours),
  // or we draw straight to the panel (also RGB565-native). We deliberately do
  // not try an 8bpp sprite: the CL_* palette is RGB565 and would mis-map to the
  // 8bpp (RGB332) colour space.
  bool ok = (canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr);
  if (ok) {
    g = &canvas; g_buffered = true;
  } else {
    g = &M5.Display; g_buffered = false;
    Serial.println("[ui] sprite alloc failed; drawing direct to panel");
    M5.Display.setTextWrap(false);
  }

  setenv("TZ", "UTC0", 1); tzset();          // work entirely in UTC

  db.begin();
  if (!cfg.load()) cfg.save();               // load persisted settings
  if (cfg.lat != 0.0 || cfg.lon != 0.0) loc.setManual(cfg.lat, cfg.lon, cfg.altM);
  pred.setSite(loc.obs());

  if (db.loadGpFromFs()) setStatus("Loaded GP: " + String(db.count()));
  loadFavsFromFile();

  // Reconnect WiFi so the App can fetch transponders from SatNOGS and NTP-sync
  // the clock. Best effort and non-fatal: everything offline still works from
  // cache. (On a deep-sleep wake this re-establishes the link quietly.)
  if (strlen(cfg.ssid) && !net.connected()) net.connect(cfg.ssid, cfg.pass, 12000);
  if (net.connected() && !timeIsSet()) net.syncTimeNtp();

  if (favN && timeIsSet()) buildSchedule();
  onActiveSatChanged();

  // Woke from the deep-sleep-until-pass timer? Land on Next Passes so the
  // imminent pass is front and centre (the AOS alarm sounds shortly after).
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
    screen = SCR_PASSES;
    if (favN && timeIsSet()) buildSchedule();
  }

  draw();
}

// ===========================================================================
//  Small helpers
// ===========================================================================
void App::setStatus(const String& s, uint32_t ms) {
  status = s; statusUntil = millis() + ms;
  Serial.printf("[status] %s\n", s.c_str());
}

time_t App::nowUtc() { return time(nullptr); }

void App::loadFavsFromFile() {
  favN = Favs::load(favs, MAX_FAVS);
  if (activeFav >= favN) activeFav = 0;
}

SatEntry* App::activeSat() {
  if (favN == 0) return nullptr;
  if (activeFav < 0 || activeFav >= favN) activeFav = 0;
  int idx = db.indexOfNorad(favs[activeFav]);
  if (idx < 0) return nullptr;
  return &db.at(idx);
}

// Load the active satellite's transponders (cache first, then SatNOGS if we
// have WiFi). Tags FM uplink CTCSS from the built-in table.
bool App::ensureTransponders(SatEntry& s) {
  activeTxCount = 0; curTx = 0;
  int n = SatDb::loadTxCache(s.norad, activeTx, MAX_TX_PER_SAT);
  if (n == 0 && net.connected()) {
    String j;
    if (net.fetchSatnogsTransmitters(s.norad, j)) {
      SatDb::saveTxCache(s.norad, j);
      n = SatDb::parseTransmittersJson(j, activeTx, MAX_TX_PER_SAT);
    }
  }
  // Apply known FM uplink tones (display only -- no radio to key).
  float tone = SatDb::knownCtcssHz(s.norad);
  if (tone > 0) for (int i = 0; i < n; ++i)
    if (activeTx[i].uplink && activeTx[i].toneHz == 0 &&
        strcasecmp(activeTx[i].mode, "FM") == 0) activeTx[i].toneHz = tone;
  activeTxCount = n;
  s.txLoaded = true;
  return n > 0;
}

void App::onActiveSatChanged() {
  SatEntry* s = activeSat();
  if (!s) { activeTxCount = 0; polarPathValid = false; return; }
  pred.setSite(loc.obs());
  pred.setSat(*s);
  ensureTransponders(*s);
  buildPolarPath();
}

// ===========================================================================
//  Next-passes schedule (one upcoming/in-progress pass per selected sat)
//  -- adapted from CardSat's buildSchedule(), favorites -> favs[]
// ===========================================================================
void App::buildSchedule() {
  schedN = 0; nextAos = 0; nextAosName[0] = 0;
  if (!timeIsSet()) return;
  time_t now = nowUtc();
  pred.setSite(loc.obs());

  // One row per selected favorite -- we never skip a satellite. If no upcoming
  // pass can be found (or its GP is missing), it still gets a row with
  // hasPass=false so all selected sats are always visible/scrollable.
  for (int i = 0; i < favN && schedN < SCHED_MAX; ++i) {
    SchedEntry e;
    e.norad = favs[i];

    int idx = db.indexOfNorad(favs[i]);
    if (idx < 0) {
      // Favorite not present in the GP catalog (e.g. dropped from the bulletin).
      snprintf(e.name, sizeof(e.name), "NORAD %lu", (unsigned long)favs[i]);
      e.hasPass = false;
      sched[schedN++] = e;
      continue;
    }
    SatEntry& s = db.at(idx);
    strncpy(e.name, s.name, sizeof(e.name) - 1); e.name[sizeof(e.name) - 1] = 0;

    if (!pred.setSat(s)) { e.hasPass = false; sched[schedN++] = e; continue; }

    LiveLook L = pred.look(now);
    if (L.el >= 0.0) {
      e.hasPass = true; e.inProgress = true; e.aos = now; e.maxEl = (float)L.el;
      time_t t = now, los = now;
      for (int k = 0; k < 120; ++k) {            // up to 60 min, 30 s steps
        t += 30; LiveLook l2 = pred.look(t);
        if (l2.el < 0.0) { los = t; break; }
        if (l2.el > e.maxEl) e.maxEl = (float)l2.el;
        los = t;
      }
      e.los = los;
    } else {
      PassPredict p;
      if (pred.predictPasses(now, cfg.minPassEl, &p, 1) >= 1) {
        e.hasPass = true; e.inProgress = false;
        e.aos = p.aos; e.los = p.los; e.maxEl = p.maxEl;
      } else {
        e.hasPass = false;                       // no pass found -> keep, show last
      }
    }
    sched[schedN++] = e;
  }

  // Sort by AOS, but push the no-pass entries to the bottom (treat them as
  // "infinitely far"). Insertion sort on a composite key.
  auto keyOf = [](const SchedEntry& e) -> time_t {
    return e.hasPass ? e.aos : (time_t)0x7FFFFFFFFFFFFFFFLL;
  };
  for (int i = 1; i < schedN; ++i) {
    SchedEntry key = sched[i]; time_t kk = keyOf(key); int j = i - 1;
    while (j >= 0 && keyOf(sched[j]) > kk) { sched[j+1] = sched[j]; --j; }
    sched[j+1] = key;
  }

  // Soonest *future* AOS feeds the alarm.
  for (int i = 0; i < schedN; ++i) {
    if (sched[i].hasPass && !sched[i].inProgress && sched[i].aos > now) {
      nextAos = sched[i].aos;
      strncpy(nextAosName, sched[i].name, sizeof(nextAosName) - 1);
      nextAosName[sizeof(nextAosName) - 1] = 0;
      break;
    }
  }
  // Restore the propagator to the active satellite for the live screens.
  SatEntry* a = activeSat(); if (a) pred.setSat(*a);
}

void App::refreshScheduleIfNeeded() {
  if (favN == 0 || !timeIsSet()) return;
  uint32_t ms = millis();
  bool due = (nextAos == 0) || (nowUtc() >= nextAos) ||
             (ms - lastSchedMs > 600000UL);       // at least every 10 min
  if (!due) return;
  buildSchedule();
  lastSchedMs = ms;
}

// ---- AOS alarm: countdown beeps + screen flash (verbatim behaviour) --------
void App::serviceAosAlarm() {
  if (!cfg.aosAlarm || !timeIsSet() || nextAos == 0) return;
  if (alarmAos != nextAos) { alarmAos = nextAos; alarmMarks = 0; }
  long dt = (long)(nextAos - nowUtc());
  if (dt <= 60 && !(alarmMarks & 1)) { alarmMarks |= 1; beep(1500, 80); }
  if (dt <= 30 && !(alarmMarks & 2)) { alarmMarks |= 2; beep(1500, 80); }
  if (dt <= 10 && !(alarmMarks & 4)) { alarmMarks |= 4; beep(1800, 90); }
  if (dt <= 0  && !(alarmMarks & 8)) {
    alarmMarks |= 8;
    beep(2600, 250); delay(120); beep(2600, 250);
    aosFlashUntil = millis() + 8000;
    strncpy(aosFlashName, nextAosName, sizeof(aosFlashName) - 1);
    aosFlashName[sizeof(aosFlashName) - 1] = 0;
    nextAos = 0;
  }
}

// ---- deep-sleep until ~60 s before the next AOS; KEY1 wakes ----------------
void App::sleepUntilNextPass() {
  if (!timeIsSet()) { setStatus("Clock not set"); }
  if (nextAos == 0) buildSchedule();

  const long lead = 60;
  long secs = (nextAos && timeIsSet()) ? ((long)(nextAos - nowUtc()) - lead) : -1;
  bool timed = (secs >= 5);
  if (timed && secs > 12L * 3600) secs = 12L * 3600;   // safety cap

  g->fillScreen(CL_BLACK);
  g->setTextSize(1);
  g->setTextColor(CL_CYAN, CL_BLACK);
  g->setCursor(6, 30);
  if (timed) g->printf("Deep sleep %ldm%02lds", secs/60, secs%60);
  else       g->print("Deep sleep");
  g->setTextColor(CL_WHITE, CL_BLACK);
  if (timed) { g->setCursor(6, 46); g->printf("until %.20s", nextAosName); }
  g->setTextColor(CL_GREY, CL_BLACK);
  g->setCursor(6, 64); g->print("Press KEY1 to wake");
  flush();
  delay(1500);

  M5.Display.sleep();
  M5.Display.waitDisplay();

  // Wake on KEY1 (front button, active-low) via ext0; and on the timer if we
  // have a scheduled pass. A KEY1 press while asleep brings us back.
  esp_sleep_enable_ext0_wakeup((gpio_num_t)Keys::wakePin(), BTN_WAKE_LEVEL);
  if (timed) esp_sleep_enable_timer_wakeup((uint64_t)secs * 1000000ULL);
  esp_deep_sleep_start();    // resets the SoC; main() + App::setup() run on wake
}

// ===========================================================================
//  Re-open the setup web portal (KEY2 long-press) so the user can change their
//  location and satellite selection without re-flashing. Requires WiFi: try
//  the saved network first, then fall back to the captive portal. On finish,
//  the setup server re-caches transponders for the (possibly new) sat list, so
//  we just reload everything from flash afterward.
// ===========================================================================
void App::reenterSetup() {
  // Show a short banner so the long-press feels acknowledged.
  g->fillScreen(CL_BLACK);
  g->setTextSize(1);
  g->setTextColor(CL_CYAN, CL_BLACK);
  g->setCursor(6, 30); g->print("Re-opening setup...");
  g->setTextColor(CL_WHITE, CL_BLACK);
  g->setCursor(6, 46); g->print("Connecting to WiFi");
  flush();

  // Free the ~64 KB off-screen sprite (if we have one) before the web server +
  // TLS run -- the no-PSRAM PICO-D4 needs that heap back for the connection and
  // the satellite-picker page. The portal draws directly to the panel anyway.
  bool hadSprite = g_buffered;
  if (hadSprite) {
    canvas.deleteSprite();
    g = &M5.Display; g_buffered = false;
    M5.Display.setTextWrap(false);
  }

  // Ensure a connection. If saved creds fail (or none), run the captive portal
  // to (re)collect them; that also lets the user move the device to a new net.
  bool haveWifi = net.connected();
  if (!haveWifi && strlen(cfg.ssid))
    haveWifi = net.connect(cfg.ssid, cfg.pass, 12000);
  if (!haveWifi)
    haveWifi = Portal::runWifiPortal(cfg, net);   // blocks until connected

  if (haveWifi && !timeIsSet()) net.syncTimeNtp();

  // Run the location + satellite picker. It writes location into cfg and the
  // selection to FILE_FAVS, then caches transponders for offline use.
  if (haveWifi) {
    Portal::runSetupServer(cfg, db, net, loc);
  } else {
    setStatus("No WiFi - setup canceled", 4000);
  }

  // Recreate the off-screen sprite now the heavy network objects are gone.
  if (hadSprite) {
    canvas.setColorDepth(16);
    if (canvas.createSprite(M5.Display.width(), M5.Display.height()) != nullptr) {
      g = &canvas; g_buffered = true;
    }
  }

  // Reload everything the setup may have changed and rebuild the live state.
  cfg.load();
  if (cfg.lat != 0.0 || cfg.lon != 0.0) loc.setManual(cfg.lat, cfg.lon, cfg.altM);
  pred.setSite(loc.obs());
  db.loadGpFromFs();
  loadFavsFromFile();
  activeFav = 0;
  if (favN && timeIsSet()) buildSchedule();
  onActiveSatChanged();

  screen = SCR_PASSES;
  lastDrawMs = 0;
  draw();
}

// ===========================================================================
//  Polar ground-track path (current pass, or next pass if not up now)
//  -- adapted verbatim from CardSat's buildPolarPath()
// ===========================================================================
void App::buildPolarPath() {
  polarPathValid = false;
  SatEntry* s = activeSat();
  if (!s || !timeIsSet()) return;
  pred.setSite(loc.obs());
  if (!pred.setSat(*s)) return;
  time_t now = nowUtc();
  PassPredict pp[3];
  int np = pred.predictPasses(now - 1800, 0.5f, pp, 3);   // include an in-progress pass
  PassPredict* use = nullptr;
  for (int i = 0; i < np; ++i) if (pp[i].los > now) { use = &pp[i]; break; }
  if (!use) return;
  polarPass = *use;
  double span = (double)(use->los - use->aos); if (span < 1) span = 1;
  for (int i = 0; i < POLAR_PTS; ++i) {
    time_t t = use->aos + (time_t)llround(span * i / (double)(POLAR_PTS - 1));
    double az, el; pred.azelAt(t, az, el);
    polarAz[i] = (float)az; polarEl[i] = (float)el;
  }
  polarPathValid = true;
}

// ===========================================================================
//  Main loop
// ===========================================================================
void App::loop() {
  M5.update();          // refresh button state for KEY1/KEY2

  handleKeys();

  refreshScheduleIfNeeded();
  serviceAosAlarm();

  // Periodic redraw so the live clock / countdown / polar marker animate.
  uint32_t ms = millis();
  uint32_t period = (screen == SCR_POLAR || screen == SCR_TRACK) ? 500 : 1000;
  if (ms - lastDrawMs > period) { lastDrawMs = ms; draw(); }

  // While an AOS alarm is flashing or counting down, animate on any screen.
  long dt = (nextAos && timeIsSet()) ? (long)(nextAos - nowUtc()) : 999999;
  bool alarmActive = (millis() < aosFlashUntil) ||
                     (cfg.aosAlarm && dt <= 60 && dt > -2);
  if (alarmActive && ms - lastDrawMs > 400) { lastDrawMs = ms; draw(); }
}

// ===========================================================================
//  Input
// ===========================================================================
void App::handleKeys() {
  // KEY1 long-press -> deep sleep (checked first so a hold never also cycles).
  if (Keys::key1Held()) { sleepUntilNextPass(); return; }

  // KEY2 long-press -> re-open the setup web portal (change sats / location).
  if (Keys::key2Held()) { reenterSetup(); return; }

  // KEY1 short click -> next screen.
  if (Keys::key1Clicked()) { nextScreen(); draw(); }

  // KEY2 short click -> advance selection (sat on PASSES/POLAR, TX on TRACK).
  if (Keys::key2Clicked()) { advanceSelection(); draw(); }
}

void App::nextScreen() {
  screen = (Screen)((screen + 1) % SCR_COUNT);
  lastDrawMs = 0;
  if (screen == SCR_POLAR) buildPolarPath();   // refresh arc on entry
}

void App::advanceSelection() {
  if (screen == SCR_TRACK) {
    if (activeTxCount > 0) { curTx = (curTx + 1) % activeTxCount; }
    return;
  }
  // PASSES / POLAR: advance to the NEXT satellite as displayed on the Next
  // Passes screen. That list is sorted by AOS time (see buildSchedule), so we
  // step through sched[] order -- not favs[] (selection / NORAD) order -- so the
  // highlight moves smoothly down the list and wraps, instead of jumping.
  if (favN == 0) return;

  uint32_t curNorad = favs[activeFav];
  if (schedN > 0) {
    // Find the current satellite's row, step to the next row (wrapping), and
    // point activeFav at whatever satellite occupies that row.
    int cur = -1;
    for (int i = 0; i < schedN; ++i) if (sched[i].norad == curNorad) { cur = i; break; }
    int next = (cur + 1) % schedN;           // if cur<0 (-1), this starts at row 0
    uint32_t nextNorad = sched[next].norad;
    for (int i = 0; i < favN; ++i) if (favs[i] == nextNorad) { activeFav = i; break; }
  } else {
    // No schedule yet (e.g. clock not set): fall back to favs[] order.
    activeFav = (activeFav + 1) % favN;
  }
  onActiveSatChanged();
}

// ===========================================================================
//  Rendering
// ===========================================================================
void App::header(const String& t) {
  g->fillRect(0, 0, 240, 16, CL_BLUE);

  // Battery indicator (top-right). getBatteryLevel() is <0 if unknown.
  int lvl = M5.Power.getBatteryLevel();
  const int bx = 216, by = 3, bw = 18, bh = 9;
  g->drawRect(bx, by, bw, bh, CL_WHITE);
  g->fillRect(bx + bw, by + 2, 2, bh - 4, CL_WHITE);
  if (lvl >= 0) {
    if (lvl > 100) lvl = 100;
    int fw = (lvl * (bw - 2)) / 100;
    uint16_t col = (lvl > 50) ? CL_GREEN : (lvl > 20 ? CL_YELLOW : CL_RED);
    if (fw > 0) g->fillRect(bx + 1, by + 1, fw, bh - 2, col);
  }

  // Clock left of the battery.
  String clk; int rightLimit = bx;
  if (timeIsSet()) {
    clk = fmtClock(nowUtc()) + "Z";
    rightLimit = bx - (int)clk.length() * 6 - 5;
  }
  const int titleX = 3, charW = 12, gap = 4;
  int maxChars = (rightLimit - gap - titleX) / charW; if (maxChars < 1) maxChars = 1;
  String title = ((int)t.length() > maxChars) ? t.substring(0, maxChars) : t;
  g->setTextColor(CL_WHITE, CL_BLUE);
  g->setTextSize(2);
  g->setCursor(titleX, 1);
  g->print(title);
  g->setTextSize(1);
  if (clk.length()) {
    g->setTextColor(CL_WHITE, CL_BLUE);
    g->setCursor(bx - (int)clk.length() * 6 - 5, 4);
    g->print(clk);
  }
}

void App::footer(const String& t) {
  g->setTextColor(CL_GREY, CL_BLACK);
  g->setTextSize(1);
  g->setCursor(2, 127);
  g->print(t);
}

void App::draw() {
  g->fillScreen(CL_BLACK);

  if (favN == 0) {
    header("StickSat");
    g->setTextSize(1);
    g->setTextColor(CL_YELLOW, CL_BLACK);
    g->setCursor(6, 40); g->print("No satellites selected.");
    g->setTextColor(CL_WHITE, CL_BLACK);
    g->setCursor(6, 54); g->print("Re-run setup over WiFi to");
    g->setCursor(6, 64); g->print("pick satellites & location.");
    flush();
    return;
  }

  switch (screen) {
    case SCR_PASSES: drawPasses(); break;
    case SCR_POLAR:  drawPolar();  break;
    case SCR_TRACK:  drawTrack();  break;
    default: break;
  }

  // transient status line
  if (status.length() && millis() < statusUntil) {
    g->fillRect(0, 114, 240, 11, CL_DGREEN);
    g->setTextColor(CL_WHITE, CL_DGREEN);
    g->setTextSize(1);
    g->setCursor(2, 115);
    g->print(status);
  }

  // AOS alarm overlay (drawn on top of any screen) -- same as CardSat.
  long dt = (nextAos && timeIsSet()) ? (long)(nextAos - nowUtc()) : 999999;
  if (millis() < aosFlashUntil) {
    bool on = ((millis() / 400) & 1);
    g->fillRect(20, 46, 200, 44, on ? CL_RED : CL_BLACK);
    g->drawRect(20, 46, 200, 44, CL_WHITE);
    g->setTextColor(CL_WHITE, on ? CL_RED : CL_BLACK);
    g->setTextSize(2); g->setCursor(34, 52); g->print("AOS!");
    g->setTextSize(1); g->setCursor(34, 74); g->printf("%.22s", aosFlashName);
  } else if (cfg.aosAlarm && dt >= 0 && dt <= 60) {
    g->fillRect(0, 16, 240, 11, CL_ORANGE);
    g->setTextColor(CL_BLACK, CL_ORANGE);
    g->setTextSize(1); g->setCursor(2, 17);
    g->printf("AOS %.14s  T-%s", nextAosName, fmtCountdown(dt).c_str());
  }

  flush();
}

// ---- Screen 1: Next Passes (CardSat schedule format) ----------------------
void App::drawPasses() {
  header("Next Passes");
  g->setTextSize(1);
  if (!timeIsSet()) {
    g->setTextColor(CL_YELLOW, CL_BLACK);
    g->setCursor(6, 42); g->print("Clock not set (need WiFi/NTP).");
    footer("KEY1 next screen");
    return;
  }
  g->setTextColor(CL_GREY, CL_BLACK);
  g->setCursor(4, 18); g->print("When    Satellite     El Len");
  if (schedN == 0) {
    g->setTextColor(CL_YELLOW, CL_BLACK);
    g->setCursor(6, 44); g->print("No passes >= min elev.");
    footer("KEY1 screen  KEY2 next sat");
    return;
  }
  time_t now = nowUtc();
  uint32_t activeNorad = (favN ? favs[activeFav] : 0);

  // Up to 9 rows fit between the header and footer; the schedule can hold up to
  // MAX_FAVS (20). Scroll a window so the active satellite is always visible:
  // find its row, then choose the top row so the active one stays on screen and
  // we never scroll past the end.
  const int VIS = 9;
  int activeRow = 0;
  for (int i = 0; i < schedN; ++i) if (sched[i].norad == activeNorad) { activeRow = i; break; }
  int top = 0;
  if (schedN > VIS) {
    top = activeRow - VIS / 2;               // try to centre the active row
    if (top < 0) top = 0;
    if (top > schedN - VIS) top = schedN - VIS;
  }

  for (int row = 0; row < VIS && (top + row) < schedN; ++row) {
    int i = top + row;
    SchedEntry& e = sched[i];
    int y = 28 + row*10;
    bool isActive = (e.norad == activeNorad);
    if (isActive) { g->fillRect(0, y-1, 240, 10, CL_GREEN);
                    g->setTextColor(CL_BLACK, CL_GREEN); }
    else if (!e.hasPass) g->setTextColor(CL_GREY, CL_BLACK);
    else g->setTextColor(e.inProgress ? CL_GREEN : CL_WHITE, CL_BLACK);
    g->setCursor(4, y);
    if (e.hasPass) {
      String when = e.inProgress ? String("NOW") : fmtCountdown((long)(e.aos - now));
      long lenMin = (e.los - e.aos) / 60;
      g->printf("%-6s %-13.13s %3.0f %2ldm", when.c_str(), e.name, e.maxEl, lenMin);
    } else {
      g->printf("%-6s %-13.13s   -   -", "--", e.name);  // no upcoming pass / GP missing
    }
    int idx = db.indexOfNorad(e.norad);
    if (idx >= 0 && gpAgeDays(db.at(idx)) >= 14) {
      g->setTextColor(CL_RED, isActive ? CL_GREEN : CL_BLACK);
      g->setCursor(226, y); g->print("!");
    }
  }

  // Scrollbar + position indicator when the list is longer than the window.
  if (schedN > VIS) {
    const int x = 236, y0 = 28, h = VIS * 10;     // track
    g->drawRect(x, y0, 3, h, CL_GREY);
    int thumbH = (h * VIS) / schedN; if (thumbH < 4) thumbH = 4;
    int thumbY = y0 + (h - thumbH) * top / (schedN - VIS);
    g->fillRect(x, thumbY, 3, thumbH, CL_CYAN);
    // "n/N" position of the active satellite, shown in the footer line.
    char pos[12]; snprintf(pos, sizeof(pos), "%d/%d", activeRow + 1, schedN);
    g->setTextColor(CL_CYAN, CL_BLACK);
    g->setCursor(208, 127); g->print(pos);
  }

  footer("KEY1 scrn KEY2 sat");
}

// ---- shared polar grid + arc (verbatim from CardSat) ----------------------
void App::drawPolarGrid(int cx, int cy, int R) {
  g->drawCircle(cx, cy, R, CL_GREY);
  g->drawCircle(cx, cy, (R*2)/3, CL_GREY);   // 30 deg
  g->drawCircle(cx, cy, R/3, CL_GREY);       // 60 deg
  g->drawPixel(cx, cy, CL_WHITE);
  g->drawLine(cx, cy - R, cx, cy + R, CL_GREY);
  g->drawLine(cx - R, cy, cx + R, cy, CL_GREY);
  g->setTextColor(CL_GREY, CL_BLACK);
  g->setCursor(cx - 2,     cy - R - 9); g->print("N");
  g->setCursor(cx - 2,     cy + R + 2); g->print("S");
  g->setCursor(cx + R + 2, cy - 3);     g->print("E");
  g->setCursor(cx - R - 8, cy - 3);     g->print("W");
}

void App::drawPolarArc(int cx, int cy, int R, const float* az, const float* el, int n) {
  auto XY = [&](int i, int& x, int& y) {
    double e = el[i]; if (e > 90) e = 90;
    double rr = R * (90.0 - e) / 90.0;
    double a  = az[i] * (M_PI / 180.0);
    x = cx + (int)lround(rr * sin(a));
    y = cy - (int)lround(rr * cos(a));
  };
  int prevx = -1, prevy = -1, firstI = -1, lastI = -1;
  for (int i = 0; i < n; ++i) {
    if (el[i] < 0) { prevx = -1; continue; }
    int px, py; XY(i, px, py);
    if (prevx >= 0) g->drawLine(prevx, prevy, px, py, CL_CYAN);
    prevx = px; prevy = py;
    if (firstI < 0) firstI = i;
    lastI = i;
  }
  if (firstI < 0) return;
  int ax, ay, lx, ly; XY(firstI, ax, ay); XY(lastI, lx, ly);
  g->drawCircle(ax, ay, 3, CL_GREEN);
  g->setTextColor(CL_GREEN, CL_BLACK);  g->setCursor(ax + 4, ay - 3); g->print("A");
  g->fillCircle(lx, ly, 2, CL_ORANGE);
  g->setTextColor(CL_ORANGE, CL_BLACK); g->setCursor(lx + 4, ly - 3); g->print("L");
  int m = (firstI + lastI) / 2, m2 = m + 1; if (m2 > lastI) m2 = lastI;
  if (m2 > m) {
    int mx, my, nx, ny; XY(m, mx, my); XY(m2, nx, ny);
    double dx = nx - mx, dy = ny - my, dl = sqrt(dx*dx + dy*dy);
    if (dl > 0.5) {
      dx /= dl; dy /= dl;
      double ex = -dy, ey = dx;
      int tx = mx + (int)lround(6*dx),  ty = my + (int)lround(6*dy);
      int b1x = mx + (int)lround(-2*dx + 3*ex), b1y = my + (int)lround(-2*dy + 3*ey);
      int b2x = mx + (int)lround(-2*dx - 3*ex), b2y = my + (int)lround(-2*dy - 3*ey);
      g->fillTriangle(tx, ty, b1x, b1y, b2x, b2y, CL_WHITE);
    }
  }
}

// ---- Screen 2: live polar plot (current pass, or next pass if below) ------
//  Task feature 8: this screen shows the current pass when the bird is up, and
//  automatically switches to the *next* pass's polar plot when it is not above
//  the horizon. buildPolarPath() already picks the in-progress pass if there is
//  one, else the next pass, so we just rebuild it when the cached pass ends.
void App::drawPolar() {
  SatEntry* s = activeSat();
  header(s ? String(s->name) : String("Polar"));
  g->setTextSize(1);
  if (!s) { footer("KEY1 next screen"); return; }

  const int cx = 66, cy = 78, R = 50;

  if (timeIsSet() && (!polarPathValid || nowUtc() > polarPass.los)) buildPolarPath();

  drawPolarGrid(cx, cy, R);
  if (polarPathValid) drawPolarArc(cx, cy, R, polarAz, polarEl, POLAR_PTS);

  LiveLook L = timeIsSet() ? pred.look(nowUtc()) : LiveLook();

  if (timeIsSet() && L.el > 0) {              // live position marker (sat is up)
    double rr = R * (90.0 - L.el) / 90.0;
    double a  = L.az * (M_PI / 180.0);
    int px = cx + (int)lround(rr * sin(a));
    int py = cy - (int)lround(rr * cos(a));
    g->drawLine(cx, cy, px, py, CL_DGREEN);
    g->fillCircle(px, py, 3, CL_GREEN);
  }
  if (timeIsSet() && L.sunEl > 0) {           // Sun glyph
    double rr = R * (90.0 - L.sunEl) / 90.0;
    double a  = L.sunAz * (M_PI / 180.0);
    int px = cx + (int)lround(rr * sin(a));
    int py = cy - (int)lround(rr * cos(a));
    g->fillCircle(px, py, 2, CL_YELLOW);
    g->drawCircle(px, py, 4, CL_YELLOW);
  }

  int rx = 128;
  g->setTextColor(L.visible ? CL_GREEN : CL_GREY, CL_BLACK);
  g->setCursor(rx, 22);
  if (L.visible)             g->print("VISIBLE");          // current pass
  else if (polarPathValid)   g->printf("next %s", fmtHM(polarPass.aos).c_str());
  else                       g->print("below horizon");
  g->setTextColor(CL_WHITE, CL_BLACK);
  g->setCursor(rx, 40); g->printf("Az  %5.1f", L.az);
  g->setCursor(rx, 52); g->printf("El  %5.1f", L.el);
  g->setCursor(rx, 64); g->printf("Rng %.0f km", L.rangeKm);
  g->setCursor(rx, 76); g->printf("%s %.3f km/s",
                  L.rangeRate >= 0 ? "away" : "appr", fabs(L.rangeRate));
  if (timeIsSet()) {
    g->setTextColor(CL_YELLOW, CL_BLACK);
    g->setCursor(rx, 88); g->printf("Sun %03.0f/%+.0f", L.sunAz, L.sunEl);
    g->setTextColor(L.sunlit ? CL_GREEN : CL_ORANGE, CL_BLACK);
    g->setCursor(rx, 100); g->print(L.sunlit ? "sat SUNLIT" : "sat ECLIPSE");
  } else {
    g->setTextColor(CL_YELLOW, CL_BLACK);
    g->setCursor(rx, 96); g->print("clock not set");
  }
  footer("KEY1 screen  KEY2 next sat");
}

// ---- Screen 3: Doppler / transponder readout (no radio/rotator/cal) -------
//  Task feature 9: like CardSat's Track screen but read-only. KEY2 cycles the
//  transponders downloaded from SatNOGS; all the same values are shown, minus
//  any calibration line (irrelevant without radio control).
void App::drawTrack() {
  SatEntry* s = activeSat();
  header(s ? String(s->name) : String("Track"));
  g->setTextSize(1);
  if (!s) { footer("KEY1 next screen"); return; }

  LiveLook L = timeIsSet() ? pred.look(nowUtc()) : LiveLook();

  // Az / El / range / range-rate.
  g->setTextColor(L.visible ? CL_GREEN : CL_GREY, CL_BLACK);
  g->setCursor(4, 20);
  g->printf("Az %5.1f  El %5.1f%s", L.az, L.el, L.visible ? " *" : "");
  { double age = gpAgeDays(*s);
    if (age >= 0) { g->setTextColor(ageColor(age), CL_BLACK);
                    g->setCursor(186, 20); g->printf("GP%4.1fd", age); } }
  g->setTextColor(CL_WHITE, CL_BLACK);
  g->setCursor(4, 31);
  g->printf("Rng %5.0fkm  Rate %+5.2f km/s", L.rangeKm, L.rangeRate);
  if (timeIsSet() && !L.sunlit) {
    g->setTextColor(CL_ORANGE, CL_BLACK);
    g->setCursor(214, 31); g->print("ECL");
  }

  // Transponder + Doppler.
  if (activeTxCount == 0) {
    g->setTextColor(CL_YELLOW, CL_BLACK);
    g->setCursor(4, 50); g->print("No transponder data.");
    g->setCursor(4, 61); g->print("Connect WiFi to fetch from");
    g->setCursor(4, 71); g->print("SatNOGS, then reselect sat.");
  } else {
    if (curTx >= activeTxCount) curTx = 0;
    Transponder& t = activeTx[curTx];
    bool linear = t.isLinear && t.bandwidth() > 0;
    uint32_t dlOp = 0, ulOp = 0, rx = 0, tx = 0;
    // For a linear transponder show the CENTRE of the passband: offset the
    // tuning by half the downlink bandwidth (passbandFreqs maps that to the
    // downlink midpoint, and to the matching uplink midpoint -- handling
    // inverting transponders correctly). Single-channel TX use offset 0.
    int32_t centreOff = linear ? (int32_t)(t.bandwidth() / 2) : 0;
    Predictor::passbandFreqs(t, centreOff, dlOp, ulOp);
    Predictor::dopplerFreqs(dlOp, ulOp, L.rangeRate, 0, 0, rx, tx);

    g->setTextColor(CL_CYAN, CL_BLACK);
    g->setCursor(4, 44);
    g->printf("TX%d/%d %s%-.16s", curTx+1, activeTxCount,
                  linear ? "[LIN] " : "", t.desc);

    // For linear transponders DN/UP are the passband-CENTRE frequencies; for
    // single-channel they are the channel frequency. RX/TX add Doppler (what a
    // radio would tune to). The "C" tag flags that these are passband centres.
    g->setTextColor(CL_WHITE, CL_BLACK);
    g->setCursor(4, 56); g->printf("DN%s%s", linear ? "c " : " ", fmtMHz(dlOp).c_str());
    g->setTextColor(CL_GREEN, CL_BLACK);
    g->setCursor(120, 56); g->printf("RX %s", fmtMHz(rx).c_str());

    g->setTextColor(CL_WHITE, CL_BLACK);
    g->setCursor(4, 67);
    if (ulOp) g->printf("UP%s%s", linear ? "c " : " ", fmtMHz(ulOp).c_str());
    else      g->print("UP  (rx only)");
    if (ulOp) {
      g->setTextColor(CL_ORANGE, CL_BLACK);
      g->setCursor(120, 67); g->printf("TX %s", fmtMHz(tx).c_str());
    }

    // Mode / inversion / bandwidth.
    g->setTextColor(CL_GREY, CL_BLACK);
    g->setCursor(4, 79);
    if (linear) g->printf("%s bw%.1fk %s", t.mode, t.bandwidth()/1000.0f,
                              t.invert ? "INV" : "");
    else        g->printf("%s", t.mode[0] ? t.mode : "single channel");

    // FM uplink PL/CTCSS tone (display only -- no radio to key).
    if (!linear && t.uplink && t.toneHz > 0) {
      g->setTextColor(CL_ORANGE, CL_BLACK);
      g->setCursor(4, 90); g->printf("PL %.1f Hz", t.toneHz);
    }

    // Doppler shift magnitude (informational).
    long dopHz = (long)rx - (long)dlOp;
    g->setTextColor(CL_GREY, CL_BLACK);
    g->setCursor(4, 101);
    g->printf("Doppler %+ld Hz", dopHz);
  }

  if (!loc.obs().valid) {
    g->setTextColor(CL_YELLOW, CL_BLACK);
    g->setCursor(4, 112); g->print("Set location in WiFi setup");
  }
  footer("KEY1 screen  KEY2 next TX");
}
