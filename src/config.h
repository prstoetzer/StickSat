#pragma once
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
#include <Arduino.h>

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
//  GPS (optional) -- M5Stack Unit GPS v1.1 on the Grove port (PORT.CUSTOM)
// ---------------------------------------------------------------------------
//  StickC Plus Grove: yellow=G32, white=G33.  GPS unit: yellow=UART_RX,
//  white=UART_TX.  Straight Grove cable => the Stick receives the unit's TX on
//  G33 and sends to the unit's RX on G32. So the ESP32 UART uses RX=33, TX=32.
//  The unit defaults to 115200 8N1, NMEA 0183. This is OPTIONAL hardware: if
//  nothing is connected, no fix is ever obtained and nothing is shown.
// ---------------------------------------------------------------------------
static constexpr int      GPS_RX_PIN = 33;   // ESP32 RX  <- GPS TX (white/G33)
static constexpr int      GPS_TX_PIN = 32;   // ESP32 TX  -> GPS RX (yellow/G32)
static constexpr uint32_t GPS_BAUD   = 115200;

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
