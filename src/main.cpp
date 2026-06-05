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
#include <M5Unified.h>
#include "config.h"
#include "storage.h"
#include "settings.h"
#include "satdb.h"
#include "net.h"
#include "location.h"
#include "buttons.h"
#include "portal.h"
#include "app.h"
#include <esp_sleep.h>

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

  // Interactive first-time setup runs ONLY when there are no saved WiFi
  // credentials yet (a genuine first run) -- and never on a deep-sleep wake.
  // On every later power-on we keep the saved credentials, try to connect, and
  // go straight to the App regardless of success. If the connection fails the
  // App shows a warning on the main screen, and a long-press of KEY2 drops into
  // setup on demand. Location and satellite selection are always preserved on
  // flash and reloaded by the App.
  bool fromSleep = (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) ||
                   (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0);

  if (!fromSleep && strlen(cfg.ssid) == 0) {
    // ---- True first run: no credentials -> captive portal + initial setup ---
    // (heap-allocated; SatDb embeds a ~22 KB catalog that must not sit on the
    // ~8 KB Arduino stack, which would overflow and watchdog-reset.)
    Net* net = new Net();
    Serial.println("[boot] first run: starting WiFi captive portal"); Serial.flush();
    Portal::runWifiPortal(cfg, *net);             // blocks until connected; saves creds
    bool haveWifi = net->connected();
    if (haveWifi) net->syncTimeNtp();

    SatDb*    db  = new SatDb(); db->begin(); db->loadGpFromFs();
    Location* loc = new Location();
    if (haveWifi) {                               // run the location + sat picker
      if (db->count() == 0) { net->fetchGpToFile(cfg.gpUrl, FILE_GP); db->loadGpFromFs(); }
      Portal::runSetupServer(cfg, *db, *net, *loc);
    }
    delete loc; delete db; delete net;
  } else if (!fromSleep) {
    // ---- Normal power-on: keep saved credentials, just try to connect -------
    // Non-blocking w.r.t. setup: success or failure, we hand off to the App.
    // (The App re-connects on its own too, but doing it here means the main
    // screen already reflects the right state on first paint.)
    Net* net = new Net();
    Serial.println("[boot] connecting to saved WiFi"); Serial.flush();
    bool haveWifi = net->connect(cfg.ssid, cfg.pass, 15000);
    Serial.printf("[boot] saved WiFi connect = %d\n", haveWifi); Serial.flush();
    if (haveWifi) net->syncTimeNtp();
    delete net;
  }
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
