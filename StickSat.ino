// StickSat.ino - single-file build (generated)
// M5StickC Plus 1.1

#include <M5Unified.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <FS.h>
#include <HTTPClient.h>
#include <LittleFS.h>
#include <Sgp4.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_sleep.h>
#include <math.h>
#include <sys/time.h>
#include <time.h>

// ====== config.h ======
// ===========================================================================
//  config.h  -  compile-time configuration and shared constants (StickSat)
// ===========================================================================
//  StickSat is a cut-down port of CardSat (the M5Cardputer-ADV satellite
//  tracker) to the M5Stack StickC Plus 1.1 (ESP32-PICO-D4, 4 MB flash, NO
//  PSRAM, ~520 KB SRAM, 135x240 ST7789 LCD, buttons A/B/C, passive buzzer).
//  It keeps CardSat's SGP4 prediction, the Next-Passes schedule, the live
//  polar plot, the Doppler readout, the AOS alarm and deep-sleep-until-pass;
//  it DROPS all CAT radio control, the antenna rotator, GPS, the mutual-window
//  finder and per-satellite calibration.
//
//  Setup is done over WiFi: a captive portal collects the WiFi credentials on
//  first boot, then an on-device web server lets the user set their location
//  and pick up to MAX_FAVS satellites from the downloaded AMSAT GP list.
//
//  RAM NOTE: the PICO-D4 has no PSRAM, so we keep the in-RAM satellite catalog
//  modest, parse the GP download by streaming it to flash, and tolerate the
//  full-frame canvas sprite (~64 KB) failing to allocate by falling back to
//  un-buffered direct drawing (see app.cpp).
// ===========================================================================

// ---- Speed of light (m/s) used for Doppler ----
static constexpr double C_LIGHT = 299792458.0;

// ---------------------------------------------------------------------------
//  Data sources (unchanged from CardSat)
// ---------------------------------------------------------------------------
//  Orbital data is GP (General Perturbations / OMM) element sets in JSON, from
//  AMSAT's distribution. Each record carries the SGP4 mean elements in named
//  fields plus an AMSAT_NAME friendly name. Transponder frequencies come from
//  the SatNOGS DB as JSON, keyed by NORAD id.
// ---------------------------------------------------------------------------
#define AMSAT_GP_URL   "https://newark192.amsat.org/gpdata/current/daily-bulletin.json"
#define SATNOGS_TX_URL "https://db.satnogs.org/api/transmitters/?format=json&satellite__norad_cat_id="

// ---------------------------------------------------------------------------
//  Buttons (M5StickC Plus 1.1 has three keys; we use the front + side ones)
// ---------------------------------------------------------------------------
//  KEY1 = Button A, front face, GPIO37
//         short press -> next screen (cycles all screens)
//         long  press -> deep sleep   (a front-key press wakes the device)
//  KEY2 = Button B, right side,  GPIO39
//         short press -> advance one satellite / transponder (wraps)
//         long  press -> re-open the setup web portal
//  (Button C / GPIO35 is the top "power" key, left unused.)
//  Read through M5Unified's debounced Button_Class: KEY1 = M5.BtnA,
//  KEY2 = M5.BtnB. BTN_FRONT_PIN is only needed for the ext0 deep-sleep wake.
//  GPIO37 is RTC-capable (RTC_GPIO5) so it is valid for ext0 wakeup, and the
//  M5StickC buttons idle HIGH (active-low), so we wake on a LOW level.
// ---------------------------------------------------------------------------
static constexpr int      BTN_FRONT_PIN  = 37;   // Button A (front)  -> KEY1
static constexpr int      BTN_SIDE_PIN   = 39;   // Button B (side)   -> KEY2
static constexpr uint32_t BTN_LONG_MS    = 700;  // hold this long = long-press
static constexpr uint8_t  BTN_WAKE_LEVEL = 0;    // active-low: wakes on LOW

// ---------------------------------------------------------------------------
//  Limits  (kept modest for the PICO-D4's RAM / 4 MB flash)
// ---------------------------------------------------------------------------
static constexpr int   MAX_SATS        = 160;  // sats held in RAM from GP data
static constexpr int   MAX_TX_PER_SAT  = 32;   // transmitters held for active sat
static constexpr int   MAX_FAVS        = 20;   // task: up to 20 selectable sats
static constexpr int   PASS_LIST_LEN   = 4;    // passes pre-computed per fav
static constexpr int   SCHED_MAX       = MAX_FAVS; // schedule rows (one per fav)
static constexpr int   POLAR_PTS       = 48;   // samples in a polar ground-track arc

// ---------------------------------------------------------------------------
//  Files on LittleFS
// ---------------------------------------------------------------------------
#define FILE_GP      "/gp.json"        // cached GP/OMM download (JSON array)
#define FILE_CFG     "/config.json"
#define FILE_TXCACHE "/tx_%lu.json"    // %lu = norad id
#define FILE_FAVS    "/favs.txt"       // favorite NORAD ids, one per line

// ====== storage.h ======
// ===========================================================================
//  storage.h -- filesystem abstraction (internal LittleFS)
// ===========================================================================
//  StickSat persists everything to internal flash via LittleFS. (CardSat also
//  carried a microSD fallback for launcher use; the StickS3 build flashes its
//  own partition table with a LittleFS data region, so the SD path is dropped.)

namespace Store {
  bool    begin();          // mount LittleFS (format on first-boot failure)
  fs::FS& fs();             // the active filesystem
  bool    ready();          // true if the filesystem mounted
  bool    format();         // wipe LittleFS (factory reset)
}

// ====== satdb.h ======
// ===========================================================================
//  satdb.h  -  in-memory satellite catalog (slim) + transponder parsing
// ===========================================================================
//  Orbital data is GP (General Perturbations / OMM) element sets, sourced from
//  AMSAT's JSON distribution. The legacy TLE *text* format is being retired as
//  the 5-digit NORAD catalog field runs out; GP/OMM carries the same SGP4 mean
//  elements in named fields with no width limit. We store the elements here and
//  reconstruct a TLE line-pair on demand only to feed the SGP4 propagator (the
//  Hopperpop library ingests elements via twoline2rv); see gpToTle().
//
//  RAM note: the StampS3A has ~512 KB internal SRAM and no PSRAM, so we keep
//  SatEntry small (no embedded transponder array). Transponders are parsed on
//  demand into a caller-supplied buffer for the *active* satellite only.
// ===========================================================================

struct Transponder {
  char     desc[40];
  uint32_t downlink     = 0; // Hz (downlink_low;  0 if none)
  uint32_t downlinkHigh = 0; // Hz (downlink_high; 0 if single-channel)
  uint32_t uplink       = 0; // Hz (uplink_low;    0 if none / beacon)
  uint32_t uplinkHigh   = 0; // Hz (uplink_high;   0 if single-channel)
  char     mode[12] = {0};   // e.g. "FM", "USB", "DATA"
  bool     invert   = false; // inverting linear transponder
  bool     isLinear = false; // true => has a tunable passband (do passband tracking)
  float    toneHz   = 0.0f;  // required FM uplink CTCSS/PL tone (0 = none)

  // Downlink passband width in Hz (0 for single-channel / FM).
  uint32_t bandwidth() const {
    return (downlinkHigh > downlink) ? (downlinkHigh - downlink) : 0;
  }
};

// One satellite's GP mean elements (the SGP4 inputs) plus identity.
struct SatEntry {
  char     name[26];          // AMSAT_NAME
  uint32_t norad = 0;         // NORAD_CAT_ID (identity / display)
  char     intlDes[12] = {0}; // OBJECT_ID, e.g. "1974-089B"
  double   epochUnix = 0;     // EPOCH as Unix UTC seconds (fractional)
  double   incl = 0;          // INCLINATION       (deg)
  double   ecc = 0;           // ECCENTRICITY      (dimensionless)
  double   raan = 0;          // RA_OF_ASC_NODE    (deg)
  double   argp = 0;          // ARG_OF_PERICENTER (deg)
  double   ma = 0;            // MEAN_ANOMALY      (deg)
  double   meanMotion = 0;    // MEAN_MOTION       (rev/day)
  double   bstar = 0;         // BSTAR             (1/earth radii)
  double   ndot = 0;          // MEAN_MOTION_DOT   (rev/day^2, = ndot/2)
  double   nddot = 0;         // MEAN_MOTION_DDOT  (rev/day^3, = nddot/6)
  uint32_t revAtEpoch = 0;    // REV_AT_EPOCH
  uint16_t elsetNum = 0;      // ELEMENT_SET_NO
  bool     txLoaded = false;  // have we fetched transponders this session?
};

class SatDb {
public:
  bool begin();                  // mount LittleFS
  int  count() const { return _n; }
  SatEntry& at(int i) { return _sats[i]; }
  int  indexOfNorad(uint32_t norad) const;

  // Parse AMSAT's GP JSON (array of OMM objects) into the catalog.
  int  loadGpFromJson(const String& json);    // replace DB
  int  appendGpFromJson(const String& json);  // append (dedup/replace by norad)
  bool loadGpFromFs();                         // reload cached GP JSON at boot
  int  loadGpFromFile(const char* path);       // stream-parse a GP file (low RAM)
  bool saveGpJson(const String& json);         // cache the downloaded blob

  // Reconstruct a TLE line-pair from a satellite's GP elements (69 chars each,
  // checksummed). Only used to initialise the SGP4 propagator. Returns false
  // on a malformed entry.
  static bool gpToTle(const SatEntry& s, char l1[72], char l2[72]);

  // Parse an OMM EPOCH string ("YYYY-MM-DD HH:MM:SS.ffffff") to Unix UTC
  // seconds (fractional). TZ-independent. Exposed for manual entry.
  static double gpEpochToUnix(const char* s);

  // Parse a SatNOGS /api/transmitters/ JSON array into out[0..maxN-1].
  // Returns number of (active) transponders parsed.
  static int parseTransmittersJson(const String& json,
                                   Transponder* out, int maxN);

  // Per-satellite transponder cache on LittleFS.
  static bool saveTxCache(uint32_t norad, const String& json);
  static int  loadTxCache(uint32_t norad, Transponder* out, int maxN);

  // Required FM uplink CTCSS (PL) tone in Hz for well-known FM satellites
  // (SatNOGS carries no structured tone field), or 0 if none/unknown.
  static float knownCtcssHz(uint32_t norad);

private:
  SatEntry _sats[MAX_SATS];
  int      _n = 0;
};

// ====== location.h ======
// ===========================================================================
//  location.h  -  observer location (manual lat/lon or Maidenhead grid)
// ===========================================================================
//  The StickSat cut-down has no GPS (CardSat's optional Grove / Cap-LoRa GNSS
//  is removed). Location is set over the web setup page as a grid square or as
//  decimal lat/lon, and persisted in Settings.

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

// ====== predict.h ======
// ===========================================================================
//  predict.h  -  SGP4 wrapper: live look-angles, Doppler range-rate, passes
// ===========================================================================

struct PassPredict {
  time_t aos = 0;        // unix UTC of acquisition of signal
  time_t los = 0;        // unix UTC of loss of signal
  time_t tca = 0;        // unix UTC of time of closest approach (max elev)
  float  maxEl = 0;      // degrees
  float  azAos = 0;
  float  azLos = 0;
};

struct LiveLook {
  double az = 0, el = 0;     // degrees
  double rangeKm = 0;        // slant range
  double rangeRate = 0;      // km/s, +ve = receding
  double subLat = 0, subLon = 0, satAltKm = 0;
  bool   visible = false;    // el > 0
  bool   sunlit = true;      // satellite illuminated (not in Earth's shadow)
  double sunAz = 0, sunEl = 0;   // Sun position from the observer (degrees)
};

// One co-visibility (mutual) window finder was in CardSat; the StickSat
// cut-down drops it (no DX-grid entry on a two-button device).

class Predictor {
public:
  void setSite(const Observer& o);
  // Point the propagator at a satellite (renders its GP elements for SGP4).
  bool setSat(SatEntry& s);

  // Compute az/el/range/range-rate at unix time `t` (UTC seconds).
  LiveLook look(time_t t);

  // Lightweight: just az/el (degrees) for the current site at time t.
  bool azelAt(time_t t, double& az, double& el);

  // Doppler-corrected radio frequencies for the current geometry.
  //   rxHz: tune the receiver here to hear a downlink of dlNominal
  //   txHz: transmit here so the satellite receives ulNominal
  static void dopplerFreqs(uint32_t dlNominal, uint32_t ulNominal,
                           double rangeRateKmS,
                           int32_t calDlHz, int32_t calUlHz,
                           uint32_t& rxHz, uint32_t& txHz);

  // Linear-transponder passband tracking. Given a tuning offset measured in Hz
  // up from the downlink passband bottom, return the *operating* downlink and
  // uplink centre frequencies (before Doppler). For an inverting transponder
  // the uplink moves opposite to the downlink; for non-inverting it tracks it.
  // Single-channel transponders ignore the offset (dlOp=downlink, ulOp=uplink).
  static void passbandFreqs(const Transponder& t, int32_t pbOffsetHz,
                            uint32_t& dlOp, uint32_t& ulOp);

  // Fill up to `maxN` upcoming passes starting from `from` (unix UTC).
  int  predictPasses(time_t from, float minEl, PassPredict* out, int maxN);

  static time_t jdToUnix(double jd);

private:
  Sgp4   _sat;
  Observer _o;
  bool   _haveSat = false;
  char   _name[26], _l1[72], _l2[72];
};

// ====== settings.h ======
// ===========================================================================
//  settings.h  -  persistent user configuration (LittleFS JSON)
// ===========================================================================
//  Trimmed from CardSat: no radio model / CI-V address / CAT baud, no rotator,
//  no GPS source, no per-satellite calibration. Just what the cut-down needs.

struct Settings {
  // WiFi (set by the captive portal on first boot)
  char     ssid[33] = "";
  char     pass[65] = "";
  // Orbital data source (GP/OMM JSON). Defaults to AMSAT.
  char     gpUrl[160] = AMSAT_GP_URL;
  // Location (set on the web setup page; grid or lat/lon)
  double   lat = 0.0, lon = 0.0, altM = 0.0;
  // Tracking
  float    minPassEl = 0.0f;   // include all passes (peak elevation >= 0 deg)
  bool     aosAlarm  = true;   // beep + flash before a favorite's AOS
  // True once the user has completed location + satellite selection at least
  // once; until then the device shows the "run setup" prompt.
  bool     setupDone = false;

  bool load();
  bool save();
};

// ====== net.h ======
// ===========================================================================
//  net.h  -  WiFi + HTTPS downloads (AMSAT GP, SatNOGS transponders)
// ===========================================================================
//  Reused from CardSat. The WiFi *scan* helper is dropped: on the StickS3 the
//  user picks the network from the captive-portal page (its own scan), not from
//  an on-device list.

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

// ====== favs.h ======
// ===========================================================================
//  favs.h  -  the user's selected satellites (NORAD ids), persisted to flash
// ===========================================================================
//  Up to MAX_FAVS satellites, one NORAD id per line in FILE_FAVS. Shared by the
//  setup web server (which writes the selection) and the app (which schedules
//  passes for them and scrolls through them with KEY2).

namespace Favs {
  int  load(uint32_t* out, int maxN);              // -> count
  bool save(const uint32_t* ids, int n);
  bool contains(const uint32_t* ids, int n, uint32_t norad);
}

// ====== buttons.h ======
// ===========================================================================
//  buttons.h  -  the two StickS3 keys, exposed as KEY1 / KEY2
// ===========================================================================
//  The task specifies two physical keys:
//    KEY1 = front key (Button A, GPIO37) : short -> next screen (cycles all)
//                                           long  -> deep sleep
//    KEY2 = side  key (Button B, GPIO39) : short -> advance satellite / TX
//                                           long  -> re-open the setup portal
//                                                     (change satellites / location)
//
//  Rather than read the GPIOs directly, these wrap M5Unified's debounced
//  Button_Class objects (M5.BtnA / M5.BtnB), which M5Unified maps to the
//  StickS3's front/side keys for us. M5.update() must be called each loop.
//
//    KEY1  ==  M5.BtnA   (front)
//    KEY2  ==  M5.BtnB   (side)
//
//  Edge helpers below return true exactly once per press, so the caller never
//  has to track button state itself.

namespace Keys {
  // Call once in setup() after M5.begin(): sets the long-press threshold used
  // by KEY1 (front) to enter deep sleep, and resolves the wake pin.
  void begin();

  // KEY1 (front). A short click cycles screens; a hold enters sleep. These are
  // mutually exclusive: a hold fires key1Held() on release and suppresses the
  // click. Both are edge events (true once).
  bool key1Clicked();   // front key short-pressed and released
  bool key1Held();      // front key held past the long-press threshold

  // KEY2 (side). A press advances the current list (sat / transponder); a long
  // press re-opens the setup web portal (change satellites / location).
  bool key2Clicked();   // side key pressed
  bool key2Held();      // side key held past the long-press threshold

  // The RTC-capable GPIO behind KEY1, for configuring ext0 wake from deep
  // sleep (so a press of the front key wakes the device).
  int  wakePin();
}

// ====== portal.h ======
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

// ====== app.h ======
// ===========================================================================
//  app.h  -  top-level application: 3-screen UI + AOS alarm + deep sleep
// ===========================================================================
//  Screens (KEY1 cycles through them in order, wrapping):
//    SCR_PASSES  : Next Passes list across all selected sats (CardSat format)
//    SCR_POLAR   : live polar plot of the current pass, or the next pass when
//                  the active satellite is below the horizon
//    SCR_TRACK   : Doppler / pass detail for the active sat's transponders
//                  (KEY2 cycles transponders); no radio, rotator or calibration
//
//  KEY2 advances the active satellite on PASSES/POLAR, or the transponder on
//  TRACK; both wrap at the end of the list.
//
//  KEY1 long-press -> deep sleep until ~60 s before the next AOS; a press of
//  KEY1 while asleep wakes the device.

enum Screen : uint8_t { SCR_PASSES = 0, SCR_POLAR, SCR_TRACK, SCR_COUNT };

// One upcoming (or in-progress) pass for a selected satellite. Every selected
// favorite gets an entry, even if no near-term pass was found (hasPass=false),
// so the Next Passes list always shows all of them.
struct SchedEntry {
  uint32_t norad = 0;
  char     name[26] = {0};
  time_t   aos = 0, los = 0;
  float    maxEl = 0;
  bool     inProgress = false;
  bool     hasPass = false;   // false => no upcoming pass found (shown at bottom)
};

class App {
public:
  void setup();
  void loop();

private:
  // subsystems
  Settings  cfg;
  SatDb     db;
  Net       net;
  Location  loc;
  Predictor pred;

  // UI state
  Screen   screen = SCR_PASSES;

  // selected satellites (the "favorites" the user picked during setup)
  uint32_t favs[MAX_FAVS];
  int      favN = 0;
  int      activeFav = 0;          // index into favs[] -> the active satellite

  // all-selected-sats schedule + AOS alarm
  SchedEntry sched[SCHED_MAX];
  int        schedN = 0;
  uint32_t   lastSchedMs = 0;
  time_t     nextAos = 0;
  char       nextAosName[26] = {0};
  time_t     alarmAos = 0;
  uint8_t    alarmMarks = 0;
  uint32_t   aosFlashUntil = 0;
  char       aosFlashName[26] = {0};

  // active-sat transponders (downlink/uplink/Doppler readout)
  Transponder activeTx[MAX_TX_PER_SAT];
  int      activeTxCount = 0;
  int      curTx = 0;              // selected transponder index

  // live polar ground-track arc (current pass, or next pass if not up now)
  PassPredict polarPass;
  float    polarAz[POLAR_PTS];
  float    polarEl[POLAR_PTS];
  bool     polarPathValid = false;

  uint32_t lastDrawMs = 0;

  // status line
  String   status;
  uint32_t statusUntil = 0;

  // ---- helpers ----
  void setStatus(const String& s, uint32_t ms = 2500);
  time_t nowUtc();
  SatEntry* activeSat();
  void loadFavsFromFile();
  bool ensureTransponders(SatEntry& s);   // load active sat's transponders
  void onActiveSatChanged();               // reload TX + rebuild polar path
  void buildSchedule();                    // next pass for every selected sat
  void buildPolarPath();                   // sample current/next pass for polar
  void refreshScheduleIfNeeded();
  void serviceAosAlarm();
  void sleepUntilNextPass();               // deep-sleep until ~60 s before AOS
  void reenterSetup();                     // KEY2 long-press: re-open setup portal
  void drawPolarGrid(int cx, int cy, int R);
  void drawPolarArc(int cx, int cy, int R, const float* az, const float* el, int n);

  // ---- input ----
  void handleKeys();
  void nextScreen();
  void advanceSelection();                 // KEY2: next sat / next transponder

  // ---- per-screen render ----
  void draw();
  void drawPasses();
  void drawPolar();
  void drawTrack();
  void header(const String& t);
  void footer(const String& t);
};

// ====== storage.cpp ======
// ===========================================================================
//  storage.cpp
// ===========================================================================

namespace Store {

static bool g_ready = false;

bool begin() {
  g_ready = false;
  // begin(true) formats the LittleFS partition if it mounts dirty / on first
  // boot, then mounts it. Requires a SPIFFS/LittleFS data partition to exist
  // in the flashed partition table (the default 8 MB table provides one).
  if (LittleFS.begin(true)) {
    g_ready = true;
    Serial.printf("[fs] LittleFS mounted (%u/%u bytes used)\n",
                  (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
    return true;
  }
  Serial.println("[fs] LittleFS mount FAILED (no data partition?)");
  return false;
}

fs::FS& fs() { return LittleFS; }
bool ready() { return g_ready; }
bool format() { return LittleFS.format(); }

} // namespace Store

// ====== satdb.cpp ======
// ===========================================================================
//  satdb.cpp
// ===========================================================================

bool SatDb::begin() {
  return Store::begin();
}

int SatDb::indexOfNorad(uint32_t norad) const {
  for (int i = 0; i < _n; ++i) if (_sats[i].norad == norad) return i;
  return -1;
}

static void rstrip(char* s) {
  int n = strlen(s);
  while (n > 0 && (s[n-1] == '\r' || s[n-1] == ' ' || s[n-1] == '\n')) s[--n] = 0;
}

// ---- EPOCH "YYYY-MM-DDTHH:MM:SS.ffffff" -> Unix UTC seconds (fractional) ----
// CelesTrak / AMSAT OMM JSON uses an ISO-8601 'T' between date and time (e.g.
// "2024-05-06T19:53:04.999776"); some sources use a space. We accept either by
// scanning the date and time parts separately rather than matching a literal
// separator -- a previous version assumed a space, which silently dropped the
// time-of-day and offset every satellite's epoch to midnight (causing pass
// times to be off by minutes). Civil-from-days (Hinnant) so it never depends on
// the process TZ. A trailing 'Z' (UTC) is harmless and ignored.
double SatDb::gpEpochToUnix(const char* s) {
  if (!s) return 0.0;
  int Y = 0, Mo = 1, D = 1, h = 0, mi = 0; double se = 0.0;
  // Date part.
  if (sscanf(s, "%d-%d-%d", &Y, &Mo, &D) < 3) return 0.0;
  // Time part: find the 'T' or space separator, then parse HH:MM:SS.fff.
  const char* tp = strchr(s, 'T');
  if (!tp) tp = strchr(s, ' ');
  if (tp) sscanf(tp + 1, "%d:%d:%lf", &h, &mi, &se);
  int y = Y - (Mo <= 2);
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (Mo + (Mo > 2 ? -3 : 9)) + 2) / 5 + D - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  long days = era * 146097 + (long)doe - 719468;
  return (double)days * 86400.0 + h * 3600 + mi * 60 + se;
}

// ---- Unix UTC seconds -> EPOCH string (for persisting manual entries) ------
static __attribute__((unused)) String unixToGpEpoch(double u) {
  time_t ip = (time_t)floor(u);
  double frac = u - (double)ip;
  struct tm tmv; gmtime_r(&ip, &tmv);
  char buf[40];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%09.6f",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday,
           tmv.tm_hour, tmv.tm_min, (double)tmv.tm_sec + frac);
  return String(buf);
}

// ===========================================================================
//  GP/OMM parsing
// ===========================================================================
// AMSAT sends the element values as JSON *strings* (e.g. "101.9903"); a few
// fields (ELEMENT_SET_NO) are numbers. Read either form without relying on the
// JSON library's string->number coercion.
static __attribute__((unused)) double jnum(JsonObjectConst o, const char* key) {
  JsonVariantConst v = o[key];
  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    return s ? strtod(s, nullptr) : 0.0;
  }
  return v.as<double>();
}

static __attribute__((unused)) bool parseGpObject(JsonObjectConst o, SatEntry& s) {
  const char* nm = o["AMSAT_NAME"] | (const char*)(o["OBJECT_NAME"] | "");
  if (!nm || !nm[0]) return false;
  strncpy(s.name, nm, sizeof(s.name) - 1); s.name[sizeof(s.name)-1] = 0; rstrip(s.name);
  const char* idd = o["OBJECT_ID"] | "";
  strncpy(s.intlDes, idd, sizeof(s.intlDes) - 1); s.intlDes[sizeof(s.intlDes)-1] = 0;

  s.norad       = (uint32_t)jnum(o, "NORAD_CAT_ID");
  const char* ep = o["EPOCH"] | "";
  s.epochUnix   = SatDb::gpEpochToUnix(ep);
  s.incl        = jnum(o, "INCLINATION");
  s.ecc         = jnum(o, "ECCENTRICITY");
  s.raan        = jnum(o, "RA_OF_ASC_NODE");
  s.argp        = jnum(o, "ARG_OF_PERICENTER");
  s.ma          = jnum(o, "MEAN_ANOMALY");
  s.meanMotion  = jnum(o, "MEAN_MOTION");
  s.bstar       = jnum(o, "BSTAR");
  s.ndot        = jnum(o, "MEAN_MOTION_DOT");
  s.nddot       = jnum(o, "MEAN_MOTION_DDOT");
  s.revAtEpoch  = (uint32_t)jnum(o, "REV_AT_EPOCH");
  s.elsetNum    = (uint16_t)jnum(o, "ELEMENT_SET_NO");
  s.txLoaded    = false;
  // A valid element set needs a non-zero epoch and mean motion.
  return s.norad != 0 && s.epochUnix > 0 && s.meanMotion > 0;
}

int SatDb::loadGpFromJson(const String& json) {
  _n = 0;
  return appendGpFromJson(json);
}

// Extract the raw value of "key" from a flat JSON object in [o, o+len). Copies
// the unquoted string value (or a bare token like 999.0 / null) into out. The
// trailing-quote check means "MEAN_MOTION" won't match "MEAN_MOTION_DOT". This
// uses no heap, unlike a per-object ArduinoJson document -- which matters
// because the GP array is parsed while a ~75 KB download buffer is resident and
// repeated document alloc/free fragments the no-PSRAM heap (it would quietly
// fail partway and drop the rest of the satellites).
static bool gpFindValue(const char* o, size_t len, const char* key,
                        char* out, size_t outsz) {
  out[0] = 0;
  size_t klen = strlen(key);
  const char* end = o + len;
  const char* hit = nullptr;
  for (const char* p = o; p + klen + 2 <= end; ++p) {
    if (*p == '"' && memcmp(p + 1, key, klen) == 0 && p[1 + klen] == '"') {
      hit = p + klen + 2; break;            // just past the key's closing quote
    }
  }
  if (!hit) return false;
  while (hit < end && (*hit == ' ' || *hit == '\t' || *hit == ':')) ++hit;
  if (hit >= end) return false;
  size_t n = 0;
  if (*hit == '"') {                        // quoted string value
    ++hit;
    while (hit < end && *hit != '"' && n + 1 < outsz) {
      if (*hit == '\\' && hit + 1 < end) ++hit;
      out[n++] = *hit++;
    }
  } else {                                  // bare token (number / null / true)
    while (hit < end && *hit != ',' && *hit != '}' &&
           *hit != ' ' && *hit != '\n' && *hit != '\r' && *hit != '\t' &&
           n + 1 < outsz)
      out[n++] = *hit++;
  }
  out[n] = 0;
  return true;
}

// Parse one flat OMM object (raw text, bounded) into a SatEntry. Same validity
// rule as parseGpObject but allocation-free, for the bulk GP-array path.
static bool parseGpObjectRaw(const char* o, size_t len, SatEntry& s) {
  char v[48];
  if (!gpFindValue(o, len, "AMSAT_NAME", v, sizeof(v)) || !v[0]) {
    if (!gpFindValue(o, len, "OBJECT_NAME", v, sizeof(v)) || !v[0]) return false;
  }
  strncpy(s.name, v, sizeof(s.name) - 1); s.name[sizeof(s.name)-1] = 0; rstrip(s.name);
  gpFindValue(o, len, "OBJECT_ID", v, sizeof(v));
  strncpy(s.intlDes, v, sizeof(s.intlDes) - 1); s.intlDes[sizeof(s.intlDes)-1] = 0;

  gpFindValue(o, len, "NORAD_CAT_ID",      v, sizeof(v)); s.norad      = (uint32_t)strtoul(v, nullptr, 10);
  gpFindValue(o, len, "EPOCH",             v, sizeof(v)); s.epochUnix  = SatDb::gpEpochToUnix(v);
  gpFindValue(o, len, "INCLINATION",       v, sizeof(v)); s.incl       = strtod(v, nullptr);
  gpFindValue(o, len, "ECCENTRICITY",      v, sizeof(v)); s.ecc        = strtod(v, nullptr);
  gpFindValue(o, len, "RA_OF_ASC_NODE",    v, sizeof(v)); s.raan       = strtod(v, nullptr);
  gpFindValue(o, len, "ARG_OF_PERICENTER", v, sizeof(v)); s.argp       = strtod(v, nullptr);
  gpFindValue(o, len, "MEAN_ANOMALY",      v, sizeof(v)); s.ma         = strtod(v, nullptr);
  gpFindValue(o, len, "MEAN_MOTION",       v, sizeof(v)); s.meanMotion = strtod(v, nullptr);
  gpFindValue(o, len, "BSTAR",             v, sizeof(v)); s.bstar      = strtod(v, nullptr);
  gpFindValue(o, len, "MEAN_MOTION_DOT",   v, sizeof(v)); s.ndot       = strtod(v, nullptr);
  gpFindValue(o, len, "MEAN_MOTION_DDOT",  v, sizeof(v)); s.nddot      = strtod(v, nullptr);
  gpFindValue(o, len, "REV_AT_EPOCH",      v, sizeof(v)); s.revAtEpoch = (uint32_t)strtoul(v, nullptr, 10);
  gpFindValue(o, len, "ELEMENT_SET_NO",    v, sizeof(v)); s.elsetNum   = (uint16_t)strtoul(v, nullptr, 10);
  s.txLoaded = false;
  return s.norad != 0 && s.epochUnix > 0 && s.meanMotion > 0;
}

int SatDb::appendGpFromJson(const String& json) {
  // Parse one OMM object at a time, allocation-free (see parseGpObjectRaw).
  // Walking object-by-object also tolerates a truncated download tail.
  const char* arr = strchr(json.c_str(), '[');
  if (!arr) return _n;
  const char* s = arr + 1;
  while (*s && _n < MAX_SATS) {
    while (*s && *s != '{' && *s != ']') ++s;     // skip whitespace/commas
    if (*s != '{') break;                          // ']' or end of input
    const char* objStart = s;
    int depth = 0; bool inStr = false, esc = false;
    const char* q = s;
    for (; *q; ++q) {                              // find the matching '}'
      char c = *q;
      if (inStr) {
        if (esc) esc = false;
        else if (c == '\\') esc = true;
        else if (c == '"')  inStr = false;
      } else if (c == '"') inStr = true;
      else if (c == '{')   ++depth;
      else if (c == '}') { if (--depth == 0) { ++q; break; } }
    }
    if (depth != 0) break;                         // truncated / malformed tail
    size_t len = (size_t)(q - objStart);

    SatEntry tmp;                                  // zero-allocation field parse
    if (parseGpObjectRaw(objStart, len, tmp)) {
      int idx = indexOfNorad(tmp.norad);           // replace if it already exists
      if (idx < 0) { idx = _n; _n++; }
      _sats[idx] = tmp;
    }
    s = q;                                         // continue after this object
  }
  return _n;
}

// (CardSat's manual-GP entry  addGp() / loadManualGpFile()  is dropped in the
//  StickSat cut-down: satellites come only from the AMSAT GP download.)

bool SatDb::saveGpJson(const String& json) {
  File f = Store::fs().open(FILE_GP, "w");
  if (!f) return false;
  f.print(json); f.close();
  return true;
}

bool SatDb::loadGpFromFs() {
  return loadGpFromFile(FILE_GP) > 0;
}

// Stream-parse a GP/OMM JSON array from a file, one object at a time, using a
// small fixed buffer. Never loads the whole file into RAM, so it works for the
// full ~75 KB amateur list on the no-PSRAM heap (where a single contiguous
// String would fail). Object state carries across read-buffer boundaries.
int SatDb::loadGpFromFile(const char* path) {
  _n = 0;
  File f = Store::fs().open(path, "r");
  if (!f) return 0;

  static const size_t OBJ_MAX = 1200;     // largest OMM object is ~800 bytes
  static char obj[OBJ_MAX];               // static: keep it off the stack
  uint8_t rd[256];
  size_t oi = 0;
  int  depth = 0;
  bool inStr = false, esc = false, collecting = false, started = false;

  int avail;
  while ((avail = f.read(rd, sizeof(rd))) > 0 && _n < MAX_SATS) {
    for (int i = 0; i < avail && _n < MAX_SATS; ++i) {
      char c = (char)rd[i];
      if (!started) { if (c == '[') started = true; continue; }
      if (!collecting) {                  // between objects: wait for '{'
        if (c == '{') { collecting = true; depth = 1; inStr = false; esc = false;
                        oi = 0; obj[oi++] = c; }
        continue;
      }
      bool overflow = (oi >= OBJ_MAX - 1);
      if (!overflow) obj[oi++] = c;
      if (inStr) {
        if (esc) esc = false;
        else if (c == '\\') esc = true;
        else if (c == '"')  inStr = false;
      } else if (c == '"') inStr = true;
      else if (c == '{')   ++depth;
      else if (c == '}') {
        if (--depth == 0) {               // object complete
          collecting = false;
          if (!overflow) {                // only parse if captured whole
            obj[oi] = 0;
            SatEntry tmp;
            if (parseGpObjectRaw(obj, oi, tmp)) {
              int idx = indexOfNorad(tmp.norad);
              if (idx < 0) { idx = _n; _n++; }
              _sats[idx] = tmp;
            }
          }
          oi = 0;
        }
      }
    }
  }
  f.close();
  return _n;
}

// ===========================================================================
//  GP elements -> TLE line-pair (only to initialise the SGP4 propagator)
// ===========================================================================
//  Field layout follows the canonical NORAD two-line spec. This is host-tested
//  by round-tripping the elements back through spec column offsets and by
//  checksum verification; SGP4 results are identical to the original element
//  set because TLE is just an alternate encoding of the same mean elements.

// Assumed-decimal exponential field (8 chars), e.g. " 71831-4", " 00000-0".
static void encExp(double v, char out[10]) {
  char s = (v < 0) ? '-' : ' ';
  double a = fabs(v);
  int e = 0;
  if (a != 0.0) {
    while (a >= 1.0) { a /= 10.0; e++; }
    while (a < 0.1)  { a *= 10.0; e--; }
  }
  long mant = llround(a * 1e5);
  if (mant >= 100000) { mant = 10000; e++; }
  if (e > 9)  e = 9;  if (e < -9) e = -9;
  snprintf(out, 10, "%c%05ld%c%01d", s, mant, (e < 0 ? '-' : '+'), (int)labs(e));
}

// First-derivative field (10 chars): sign + ".XXXXXXXX".
static void encNdot(double v, char out[12]) {
  char s = (v < 0) ? '-' : ' ';
  long m = llround(fabs(v) * 1e8);
  if (m > 99999999L) m = 99999999L;
  snprintf(out, 12, "%c.%08ld", s, m);
}

// Catalog number: 5 digits, or Alpha-5 for 100000-339999 (TLE's stopgap).
static void encCatalog(uint32_t n, char out[6]) {
  if (n <= 99999u) { snprintf(out, 6, "%05lu", (unsigned long)n); return; }
  static const char* A = "ABCDEFGHJKLMNPQRSTUVWXYZ";   // skips I and O
  int hi = (int)(n / 10000), lo = (int)(n % 10000);
  if (hi >= 10 && hi <= 33) snprintf(out, 6, "%c%04d", A[hi - 10], lo);
  else snprintf(out, 6, "%05lu", (unsigned long)(n % 100000u));
}

static int tleChecksum(const char* line) {
  int s = 0;
  for (int i = 0; i < 68 && line[i]; i++) {
    char c = line[i];
    if (c >= '0' && c <= '9') s += c - '0';
    else if (c == '-')        s += 1;
  }
  return s % 10;
}

static void putAt(char* line, int col1, const char* s) {   // col1 is 1-indexed
  int i = col1 - 1;
  for (int k = 0; s[k]; k++) line[i + k] = s[k];
}

bool SatDb::gpToTle(const SatEntry& s, char l1[72], char l2[72]) {
  if (s.meanMotion <= 0 || s.epochUnix <= 0) return false;
  memset(l1, ' ', 69); l1[69] = 0;
  memset(l2, ' ', 69); l2[69] = 0;

  char cat[6]; encCatalog(s.norad, cat);

  // International designator OBJECT_ID "YYYY-NNNP[PP]" -> "YYNNNPPP".
  char intl[9] = "        ";
  if (s.intlDes[0] && strlen(s.intlDes) >= 8 && s.intlDes[4] == '-') {
    intl[0] = s.intlDes[2]; intl[1] = s.intlDes[3];
    intl[2] = s.intlDes[5]; intl[3] = s.intlDes[6]; intl[4] = s.intlDes[7];
    int k = 5;
    for (size_t j = 8; j < strlen(s.intlDes) && k < 8; ++j) intl[k++] = s.intlDes[j];
  }

  // Epoch -> YYDDD.DDDDDDDD.
  time_t ip = (time_t)floor(s.epochUnix);
  double frac = s.epochUnix - (double)ip;
  struct tm tmv; gmtime_r(&ip, &tmv);
  double day = (tmv.tm_yday + 1)
             + (tmv.tm_hour * 3600 + tmv.tm_min * 60 + tmv.tm_sec + frac) / 86400.0;
  char epoch[16];
  snprintf(epoch, sizeof(epoch), "%02d%012.8f", tmv.tm_year % 100, day);

  char nd[12];  encNdot(s.ndot, nd);
  char ndd[10]; encExp(s.nddot, ndd);
  char bs[10];  encExp(s.bstar, bs);

  // --- line 1 ---
  l1[0] = '1'; putAt(l1, 3, cat); l1[7] = 'U';
  putAt(l1, 10, intl);
  putAt(l1, 19, epoch);
  putAt(l1, 34, nd);
  putAt(l1, 45, ndd);
  putAt(l1, 54, bs);
  l1[62] = '0';                                   // ephemeris type
  char es[6]; snprintf(es, sizeof(es), "%4u", (unsigned)(s.elsetNum % 10000));
  putAt(l1, 65, es);
  l1[68] = '0' + tleChecksum(l1);

  // --- line 2 ---
  char buf[16];
  l2[0] = '2'; putAt(l2, 3, cat);
  snprintf(buf, sizeof(buf), "%8.4f", s.incl); putAt(l2, 9,  buf);
  snprintf(buf, sizeof(buf), "%8.4f", s.raan); putAt(l2, 18, buf);
  long e7 = llround(s.ecc * 1e7); if (e7 < 0) e7 = 0; if (e7 > 9999999L) e7 = 9999999L;
  snprintf(buf, sizeof(buf), "%07ld", e7);     putAt(l2, 27, buf);
  snprintf(buf, sizeof(buf), "%8.4f", s.argp); putAt(l2, 35, buf);
  snprintf(buf, sizeof(buf), "%8.4f", s.ma);   putAt(l2, 44, buf);
  snprintf(buf, sizeof(buf), "%11.8f", s.meanMotion); putAt(l2, 53, buf);
  snprintf(buf, sizeof(buf), "%5lu", (unsigned long)(s.revAtEpoch % 100000u));
  putAt(l2, 64, buf);
  l2[68] = '0' + tleChecksum(l2);
  return true;
}

// --- SatNOGS transmitters JSON -------------------------------------------
int SatDb::parseTransmittersJson(const String& json, Transponder* out, int maxN) {
  JsonDocument filter;
  JsonObject fe = filter.add<JsonObject>();
  fe["description"]   = true;
  fe["uplink_low"]    = true;
  fe["uplink_high"]   = true;
  fe["downlink_low"]  = true;
  fe["downlink_high"] = true;
  fe["mode"]          = true;
  fe["invert"]        = true;
  fe["type"]          = true;
  fe["status"]        = true;
  fe["alive"]         = true;

  JsonDocument doc;
  if (deserializeJson(doc, json, DeserializationOption::Filter(filter))) return 0;

  int n = 0;
  for (JsonObject o : doc.as<JsonArray>()) {
    if (n >= maxN) break;
    const char* st = o["status"] | "";
    bool alive = o["alive"] | true;
    if (!alive || (st[0] && strcmp(st, "active") != 0)) continue; // active only

    Transponder& t = out[n];
    const char* d = o["description"] | "";
    strncpy(t.desc, d, sizeof(t.desc)-1); t.desc[sizeof(t.desc)-1]=0;
    const char* m = o["mode"] | "";
    strncpy(t.mode, m, sizeof(t.mode)-1); t.mode[sizeof(t.mode)-1]=0;
    t.downlink     = o["downlink_low"]   | 0u;
    t.downlinkHigh = o["downlink_high"]  | 0u;
    t.uplink       = o["uplink_low"]      | 0u;
    t.uplinkHigh   = o["uplink_high"]     | 0u;
    t.invert       = o["invert"]          | false;

    const char* ty = o["type"] | "";
    bool typeLinear = (strcmp(ty, "Transponder") == 0);
    t.isLinear = (t.uplink != 0) && (t.downlinkHigh > t.downlink) &&
                 (typeLinear || (t.downlinkHigh - t.downlink) >= 5000u);
    n++;
  }
  return n;
}

static String txPath(uint32_t norad) {
  char buf[32]; snprintf(buf, sizeof(buf), FILE_TXCACHE, (unsigned long)norad);
  return String(buf);
}

bool SatDb::saveTxCache(uint32_t norad, const String& json) {
  File f = Store::fs().open(txPath(norad), "w");
  if (!f) return false;
  f.print(json); f.close();
  return true;
}

int SatDb::loadTxCache(uint32_t norad, Transponder* out, int maxN) {
  File f = Store::fs().open(txPath(norad), "r");
  if (!f) return 0;
  String j = f.readString(); f.close();
  return parseTransmittersJson(j, out, maxN);
}

// Required FM-uplink CTCSS (PL) tones for the common FM birds. SatNOGS has no
// structured tone field, so these are built in by NORAD id. Operating tones
// only (e.g. SO-50's 74.4 Hz arming burst is a separate manual action; its
// working uplink tone is 67.0 Hz). Extend as new FM satellites appear.
float SatDb::knownCtcssHz(uint32_t norad) {
  switch (norad) {
    case 25544: return 67.0f;   // ISS (FM cross-band repeater)
    case 27607: return 67.0f;   // SO-50  (SaudiSat-1C)
    case 43017: return 67.0f;   // AO-91  (RadFxSat / Fox-1B)
    case 43137: return 67.0f;   // AO-92  (Fox-1D)
    case 43678: return 141.3f;  // PO-101 (Diwata-2)
    default:    return 0.0f;
  }
}

// ====== location.cpp ======
// ===========================================================================
//  location.cpp
// ===========================================================================

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

// ====== predict.cpp ======
// ===========================================================================
//  predict.cpp
// ===========================================================================

static const double DEG = M_PI / 180.0;
static const double RE_KM = 6378.137;          // WGS84 equatorial radius

// Geocentric unit vector to the Sun in equatorial inertial coords (ECI).
// Low-precision almanac, good to ~0.01 deg -- ample for shadow / az-el.
static void sunEciUnit(double jd, double& x, double& y, double& z) {
  double n   = jd - 2451545.0;
  double L   = fmod(280.460 + 0.9856474 * n, 360.0);   // mean longitude
  double g   = fmod(357.528 + 0.9856003 * n, 360.0) * DEG;
  double lam = (L + 1.915 * sin(g) + 0.020 * sin(2 * g)) * DEG;  // ecliptic lon
  double eps = (23.439 - 0.0000004 * n) * DEG;          // obliquity
  x = cos(lam);
  y = cos(eps) * sin(lam);
  z = sin(eps) * sin(lam);
}

// Greenwich mean sidereal time (radians) for a given Julian date.
static double gmstRad(double jd) {
  double T = (jd - 2451545.0) / 36525.0;
  double g = 280.46061837 + 360.98564736629 * (jd - 2451545.0)
             + 0.000387933 * T * T - T * T * T / 38710000.0;
  g = fmod(g, 360.0); if (g < 0) g += 360.0;
  return g * DEG;
}

void Predictor::setSite(const Observer& o) {
  _o = o;
  _sat.site(o.lat, o.lon, o.altM);
}

bool Predictor::setSat(SatEntry& s) {
  strncpy(_name, s.name, sizeof(_name)-1); _name[sizeof(_name)-1]=0;
  // The SGP4 library ingests elements through twoline2rv, so render the stored
  // GP mean elements into a TLE line-pair (SGP4 is encoding-agnostic).
  if (!SatDb::gpToTle(s, _l1, _l2)) { _haveSat = false; return false; }
  _sat.init(_name, _l1, _l2);
  _haveSat = (_sat.satrec.error == 0);
  return _haveSat;
}

LiveLook Predictor::look(time_t t) {
  LiveLook L;
  if (!_haveSat) return L;

  // Range rate via central finite difference of slant range (2 s baseline).
  _sat.findsat((unsigned long)(t - 1));
  double d0 = _sat.satDist;
  _sat.findsat((unsigned long)(t + 1));
  double d1 = _sat.satDist;
  L.rangeRate = (d1 - d0) / 2.0;          // km/s

  // Current sample.
  _sat.findsat((unsigned long)t);
  L.az       = _sat.satAz;
  L.el       = _sat.satEl;
  L.rangeKm  = _sat.satDist;
  L.subLat   = _sat.satLat;
  L.subLon   = _sat.satLon;
  L.satAltKm = _sat.satAlt;
  L.visible  = (_sat.satEl > 0.0);

  // ---- Sun geometry: satellite illumination + Sun look-angle --------------
  double jd = (double)t / 86400.0 + 2440587.5;
  double sx, sy, sz; sunEciUnit(jd, sx, sy, sz);    // Sun unit vector (ECI)
  double th = gmstRad(jd);
  double ct = cos(th), st = sin(th);

  // Satellite ECEF position from its geodetic sub-point (lat/lon/alt).
  double phi = L.subLat * DEG, lam = L.subLon * DEG, h = L.satAltKm;
  double e2 = 6.69437999014e-3;                      // WGS84 first ecc^2
  double sphi = sin(phi), cphi = cos(phi);
  double Nlat = RE_KM / sqrt(1.0 - e2 * sphi * sphi);
  double rx = (Nlat + h) * cphi * cos(lam);
  double ry = (Nlat + h) * cphi * sin(lam);
  double rz = (Nlat * (1.0 - e2) + h) * sphi;

  // Sun unit vector rotated ECI -> ECEF (Rz(-theta)).
  double ux =  sx * ct + sy * st;
  double uy = -sx * st + sy * ct;
  double uz =  sz;

  // Cylindrical-shadow test: in eclipse if on the anti-solar side and the
  // perpendicular distance to the Earth-Sun axis is less than Earth's radius.
  double proj = rx * ux + ry * uy + rz * uz;         // km along Sun direction
  double rmag2 = rx * rx + ry * ry + rz * rz;
  double perp  = sqrt(fmax(0.0, rmag2 - proj * proj));
  L.sunlit = !(proj < 0.0 && perp < RE_KM);

  // Sun az/el for the observer (topocentric ENU; solar parallax negligible).
  double olat = _o.lat * DEG;
  double ost = sin(th + _o.lon * DEG), oct = cos(th + _o.lon * DEG);
  double slat = sin(olat), clat = cos(olat);
  // East, North, Up (ECI) dotted with Sun unit vector:
  double eComp = (-ost) * sx + (oct) * sy;
  double nComp = (-slat * oct) * sx + (-slat * ost) * sy + (clat) * sz;
  double uComp = ( clat * oct) * sx + ( clat * ost) * sy + (slat) * sz;
  L.sunEl = atan2(uComp, sqrt(eComp * eComp + nComp * nComp)) / DEG;
  double az = atan2(eComp, nComp) / DEG; if (az < 0) az += 360.0;
  L.sunAz = az;
  return L;
}

void Predictor::dopplerFreqs(uint32_t dlNominal, uint32_t ulNominal,
                             double rangeRateKmS,
                             int32_t calDlHz, int32_t calUlHz,
                             uint32_t& rxHz, uint32_t& txHz) {
  double rr = rangeRateKmS * 1000.0;       // m/s, +ve receding
  double beta = rr / C_LIGHT;

  // Downlink: observer receives dl*(1 - beta) -> tune RX there.
  double rx = (double)dlNominal * (1.0 - beta) + (double)calDlHz;
  // Uplink: transmit so the satellite hears ul nominal -> ul/(1 - beta).
  double tx = (ulNominal ? ((double)ulNominal / (1.0 - beta)) : 0.0);
  if (ulNominal) tx += (double)calUlHz;

  rxHz = (uint32_t)llround(rx);
  txHz = (uint32_t)llround(tx);
}

void Predictor::passbandFreqs(const Transponder& t, int32_t pbOffsetHz,
                              uint32_t& dlOp, uint32_t& ulOp) {
  // No tunable downlink passband -> single channel; ignore the offset.
  uint32_t dlBw = t.bandwidth();
  if (!t.isLinear || dlBw == 0) {
    dlOp = t.downlink;
    ulOp = t.uplink;
    return;
  }

  // Clamp the tuning offset into [0, downlink bandwidth].
  int32_t off = pbOffsetHz;
  if (off < 0) off = 0;
  if ((uint32_t)off > dlBw) off = (int32_t)dlBw;

  dlOp = t.downlink + (uint32_t)off;

  if (t.uplink == 0) { ulOp = 0; return; }

  // Assume equal up/down passband width when the uplink top edge is missing.
  uint32_t ulBw = (t.uplinkHigh > t.uplink) ? (t.uplinkHigh - t.uplink) : dlBw;
  if (t.invert) {
    // Inverting: bottom of uplink maps to top of downlink. As the downlink
    // tunes up by `off`, the uplink tunes down by the same amount.
    ulOp = t.uplink + ulBw - (uint32_t)off;
  } else {
    ulOp = t.uplink + (uint32_t)off;
  }
}

time_t Predictor::jdToUnix(double jd) {
  return (time_t)llround((jd - 2440587.5) * 86400.0);
}

bool Predictor::azelAt(time_t t, double& az, double& el) {
  if (!_haveSat) { az = el = 0; return false; }
  _sat.findsat((unsigned long)t);
  az = _sat.satAz;
  el = _sat.satEl;
  return true;
}

// (CardSat's mutual/co-visibility window finder is dropped in StickSat.)

int Predictor::predictPasses(time_t from, float minEl, PassPredict* out, int maxN) {
  if (!_haveSat) return 0;
  passinfo overpass;
  _sat.initpredpoint((unsigned long)from, (double)minEl);

  int found = 0;
  for (int i = 0; i < maxN; ++i) {
    // search up to ~ a number of iterations for the next pass
    bool ok = _sat.nextpass(&overpass, 200);
    if (!ok) break;
    PassPredict& p = out[found];
    p.aos   = jdToUnix(overpass.jdstart);
    p.los   = jdToUnix(overpass.jdstop);
    p.tca   = jdToUnix(overpass.jdmax);
    p.maxEl = (float)overpass.maxelevation;
    p.azAos = (float)overpass.azstart;
    p.azLos = (float)overpass.azstop;
    found++;
  }
  return found;
}

// ====== settings.cpp ======
// ===========================================================================
//  settings.cpp
// ===========================================================================

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

// ====== net.cpp ======
// ===========================================================================
//  net.cpp  -  WiFi + HTTPS (reused from CardSat, scan helper removed)
// ===========================================================================

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

// ====== favs.cpp ======
// ===========================================================================
//  favs.cpp
// ===========================================================================

namespace Favs {

int load(uint32_t* out, int maxN) {
  File f = Store::fs().open(FILE_FAVS, "r");
  if (!f) return 0;
  int n = 0;
  while (f.available() && n < maxN) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    uint32_t id = (uint32_t)strtoul(line.c_str(), nullptr, 10);
    if (id == 0) continue;
    // de-dup
    bool dup = false;
    for (int i = 0; i < n; ++i) if (out[i] == id) { dup = true; break; }
    if (!dup) out[n++] = id;
  }
  f.close();
  return n;
}

bool save(const uint32_t* ids, int n) {
  File f = Store::fs().open(FILE_FAVS, "w");
  if (!f) return false;
  for (int i = 0; i < n; ++i) f.printf("%lu\n", (unsigned long)ids[i]);
  f.close();
  return true;
}

bool contains(const uint32_t* ids, int n, uint32_t norad) {
  for (int i = 0; i < n; ++i) if (ids[i] == norad) return true;
  return false;
}

} // namespace Favs

// ====== buttons.cpp ======
// ===========================================================================
//  buttons.cpp
// ===========================================================================

namespace Keys {

static int s_wakePin = BTN_FRONT_PIN;   // fallback to the documented G11

void begin() {
  // M5Unified's Button_Class already debounces. Set the hold threshold that
  // KEY1 (front) uses to mean "deep sleep" and KEY2 (side) uses to mean
  // "re-open setup", rather than their short-press actions.
  M5.BtnA.setHoldThresh(BTN_LONG_MS);
  M5.BtnB.setHoldThresh(BTN_LONG_MS);

  // ext0 deep-sleep wake targets the front-button GPIO (Button A = GPIO37 on
  // the M5StickC Plus 1.1, which is RTC-capable as required for ext0). If a
  // board revision differs, change BTN_FRONT_PIN in config.h.
  s_wakePin = BTN_FRONT_PIN;
  Serial.printf("[keys] KEY1=BtnA(front,G%d wake)  KEY2=BtnB(side)\n", s_wakePin);
}

// KEY1 short click: wasClicked() fires on release only when the press was
// shorter than the hold threshold, so it never collides with key1Held().
bool key1Clicked() { return M5.BtnA.wasClicked(); }

// KEY1 long press: wasHold() fires once when the press crosses the threshold.
bool key1Held() { return M5.BtnA.wasHold(); }

// KEY2 short click: side key pressed briefly and released.
bool key2Clicked() { return M5.BtnB.wasClicked(); }

// KEY2 long press: held past the threshold -> re-open setup.
bool key2Held() { return M5.BtnB.wasHold(); }

int wakePin() { return s_wakePin; }

} // namespace Keys

// ====== portal.cpp ======
// ===========================================================================
//  portal.cpp  -  captive WiFi portal + on-device setup web server
// ===========================================================================

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

// ====== app.cpp ======
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

// ====== main.cpp ======
// ===========================================================================
//  main.cpp  -  StickSat entry point
// ===========================================================================
//  Boot flow:
//    1. M5 + filesystem + settings.
//    2. If no WiFi credentials -> captive portal to collect them.
//    3. Connect to WiFi (best effort). On first connect, NTP-sync the clock.
//    4. If GP data missing or setup not finished -> run the setup web server
//       (download AMSAT GP, set location, pick up to 20 satellites).
//    5. Hand off to App, which owns the 3 live screens + AOS alarm + sleep.
//
//  A wake from deep sleep (KEY1 or the pass timer) re-enters setup() but skips
//  the portal/setup phases because credentials + setupDone are already saved,
//  landing straight in the App.

// App owns heavy library objects (the SGP4 propagator, the satellite catalog,
// the WiFi/Net helper). Constructing those during C++ static initialization --
// before the runtime, serial and M5 hardware are up -- can fault and produce a
// silent watchdog boot loop. So App is created on the heap inside setup(),
// AFTER M5.begin()/Serial.begin(), where every constructor is safe to run.
static App* app = nullptr;

void setup() {
  auto m5cfg = M5.config();
  M5.begin(m5cfg);
  Serial.begin(115200);
  delay(1500);                                // let USB-serial re-attach after reset
  Serial.println("\n[boot] StickSat starting"); Serial.flush();
  M5.Display.setRotation(1);

  // Earliest possible visible sign of life: a banner drawn straight to the
  // panel, before any subsystem that could hang. If you see this but nothing
  // else, the stall is in one of the steps below.
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.setCursor(6, 6);
  M5.Display.print("StickSat");
  M5.Display.setTextSize(1);
  M5.Display.setCursor(6, 30);
  M5.Display.print("booting...");

  Serial.println("[boot] keys");
  Keys::begin();

  Serial.println("[boot] fs");
  if (!Store::begin()) {
    // LittleFS could not mount/format (wrong partition table, etc). Don't wedge
    // -- show the error and keep going; settings just won't persist.
    M5.Display.setCursor(6, 44);
    M5.Display.setTextColor(TFT_RED, TFT_BLACK);
    M5.Display.print("FS mount failed");
    Serial.println("[boot] FS FAILED - continuing without persistence");
  }

  Serial.println("[boot] settings");
  Settings cfg;
  if (!cfg.load()) { cfg.save(); }   // first boot: write defaults
  Serial.printf("[boot] ssid='%s' setupDone=%d\n", cfg.ssid, cfg.setupDone);

  // Only run the interactive setup phases on a *fresh* boot, not on a deep-
  // sleep wake (where credentials and selection already exist).
  bool fromSleep = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) ||
                   (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0);

  if (!fromSleep) {
    // NOTE: these helpers are heap-allocated, not stack locals. SatDb embeds a
    // ~22 KB satellite catalog (SatEntry _sats[MAX_SATS]); a stack-local SatDb
    // would overflow the ~8 KB Arduino loopTask stack and trigger a watchdog
    // reset before setup() got anywhere. new/delete puts them on the heap.
    Net*      net = new Net();
    bool      haveWifi = false;

    // ---- Phase 1: WiFi credentials (captive portal if none / if they fail) --
    if (strlen(cfg.ssid) == 0) {
      Serial.println("[boot] starting WiFi captive portal"); Serial.flush();
      Portal::runWifiPortal(cfg, *net);         // saves cfg.ssid/pass on success
      haveWifi = net->connected();
    } else {
      Serial.println("[boot] connecting to saved WiFi"); Serial.flush();
      haveWifi = net->connect(cfg.ssid, cfg.pass, 15000);
      if (!haveWifi) {                          // saved creds failed -> portal
        Portal::runWifiPortal(cfg, *net);
        haveWifi = net->connected();
      }
    }
    Serial.printf("[boot] WiFi phase done (connected=%d)\n", haveWifi); Serial.flush();

    // ---- Time sync once we have a connection --------------------------------
    if (haveWifi) net->syncTimeNtp();

    // ---- Phase 2: location + satellite picker -------------------------------
    SatDb*    db = new SatDb();   // ~22 KB catalog -> heap, never the stack
    db->begin();
    db->loadGpFromFs();                         // cached GP, if any
    Location* loc = new Location();

    // Always offer setup on first boot, or whenever it hasn't been completed,
    // or when there's no GP data yet. (A finished unit skips straight to App.)
    bool needSetup = !cfg.setupDone || (db->count() == 0);
    if (haveWifi && needSetup) {
      // If GP is missing, grab it before showing the picker so the list isn't
      // empty (the setup page also has a manual re-download button).
      if (db->count() == 0) { net->fetchGpToFile(cfg.gpUrl, FILE_GP); db->loadGpFromFs(); }
      Portal::runSetupServer(cfg, *db, *net, *loc);
    }

    // Free the boot-phase helpers before App allocates its own copies; App
    // re-reads everything (settings/favs/GP/transponders) from flash.
    delete loc; delete db; delete net;
  }

  // ---- Hand off to the App (re-loads settings/favs/GP from flash) ----------
  Serial.println("[boot] creating App");
  app = new App();
  Serial.println("[boot] App created; running setup");
  app->setup();
}

void loop() {
  if (app) app->loop();
}
