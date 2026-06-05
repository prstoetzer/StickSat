#pragma once
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
#include <Arduino.h>
#include "settings.h"
#include "satdb.h"
#include "net.h"
#include "location.h"
#include "predict.h"

enum Screen : uint8_t { SCR_PASSES = 0, SCR_POLAR, SCR_TRACK, SCR_COUNT };

// One upcoming (or in-progress) pass for a selected satellite.
struct SchedEntry {
  uint32_t norad = 0;
  char     name[26] = {0};
  time_t   aos = 0, los = 0;
  float    maxEl = 0;
  bool     inProgress = false;
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
