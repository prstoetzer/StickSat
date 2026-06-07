# StickSat — a cut-down CardSat satellite tracker for the M5StickC Plus 1.1

StickSat is a slimmed-down port of **CardSat** (the M5Cardputer-ADV amateur-radio
satellite tracker) to the **M5StickC Plus 1.1** (ESP32-PICO-D4, 4 MB flash, no
PSRAM, ~520 KB SRAM, 135×240 LCD, buttons A/B/C). It keeps CardSat's offline SGP4
pass prediction, the **Next-Passes** schedule, the **live polar plot**, a
**Doppler / transponder readout**, the **AOS alarm**, and
**deep-sleep-until-pass**. It **drops** the things that need more I/O than the
Stick has: CAT radio control, the antenna rotator, per-satellite calibration,
the mutual-window finder, and manual TLE entry.

Time and location can come from three sources, in order of convenience: an
**optional Grove GPS** (auto-detected), **WiFi** (NTP for the clock + a web page
for location), or **manual entry** on the device. Once set, everything is cached
to flash so the tracker runs fully offline. You pick up to **20 satellites**
from the AMSAT distribution during setup.

---

## Controls (two buttons)

The M5StickC Plus has three keys; StickSat uses the front and side ones,
exposed in the firmware as **KEY1** (Button A, front, GPIO37) and **KEY2**
(Button B, side, GPIO39). They are read through M5Unified's debounced button
class (`M5.BtnA` = KEY1, `M5.BtnB` = KEY2). The top Button C (GPIO35) is unused.

| Action | Result |
|---|---|
| **KEY1 short press** | Next screen. Cycles **Next Passes → Polar → Track → GPS** and wraps. |
| **KEY1 long press** (≥0.7 s) | Enter **deep sleep** until ~60 s before the next AOS. |
| **KEY1 while asleep** | **Wake** the device. |
| **KEY2 short press** (Next Passes / Polar) | Advance to the **next satellite**; wraps at the end of your list. |
| **KEY2 short press** (Track) | Advance to the **next transponder** of the active satellite; wraps. |
| **KEY2 long press** (≥0.7 s, any screen) | Open the **setup chooser** — then **KEY1** = WiFi setup (location & satellites), **KEY2** = set the clock manually (no WiFi). |

On the **Next Passes** screen, when KEY2 steps past the bottom of the visible
window the list scrolls to follow the selected satellite (a scrollbar and an
`n/N` position indicator appear once you have more than nine), so all 20 favorites
are reachable.

---

## Screens

1. **Next Passes** *(home / first screen)* — one schedule across all your
   selected satellites, soonest AOS first, in the same layout CardSat uses:
   `When  Satellite  El  Len`, with in-progress passes shown as **NOW** and a
   red `!` flag on stale element sets. Every selected satellite gets a row (ones
   with no upcoming pass are listed last with `--`), and the list scrolls so all
   20 are reachable. The currently-selected satellite is highlighted. This is
   also where a deep-sleep wake lands you.

2. **Polar** — a live sky plot of the **current** pass (ground-track arc with
   AOS/LOS markers, travel-direction arrow, a live position dot and a Sun
   glyph). When the active satellite is **below the horizon**, it automatically
   shows the **next** pass's polar track instead. Right-hand readout: az/el,
   range, range-rate, the pass's **AOS/LOS times** with a countdown, peak
   elevation, and sunlit/eclipse.

3. **Track** — CardSat's Doppler/transponder detail, **read-only** (no radio,
   rotator, or calibration). Shows az/el/range/range-rate, GP age, and for the
   selected transponder the **DN/UP** centre frequencies (passband centre for
   linear transponders), the **Doppler-corrected RX/TX** a radio would use, the
   mode/inversion/bandwidth, any FM uplink PL tone, and the current Doppler
   shift in Hz. **KEY2** cycles through the transponders downloaded from SatNOGS.

4. **GPS** — status of the optional Grove GPS: fix state and satellite count,
   **latitude / longitude**, **altitude**, the **6-digit Maidenhead grid**, and
   **UTC date/time** from the unit. With no unit attached it says so and (if the
   clock is set from another source) still shows current UTC.

The **AOS alarm** (countdown beeps at T-60/30/10 s and a flashing AOS banner)
and the orange T-minus strip overlay on every screen, exactly as in CardSat.

---

## First-boot setup

The interactive setup below runs **only on the very first boot** (when no WiFi
credentials are stored yet). After that, credentials, location, and your
satellite selection are saved on flash and StickSat goes straight to tracking on
every power-on — see "Normal power-on" below.

1. Power on. With no saved WiFi, StickSat starts a SoftAP **`StickSat-Setup`**
   with a captive portal. Join it from a phone/laptop; the setup page should pop
   up automatically (or browse to `192.168.4.1`). Pick your 2.4 GHz network and
   enter the password. On success the credentials are saved and the device
   connects.
2. Once online, StickSat downloads the AMSAT GP element set and serves a setup
   page at the IP shown on screen. There you:
   - set your **location** (Maidenhead grid such as `FN31pr`, or decimal
     lat/lon),
   - set the **minimum pass elevation** (0° includes passes down to the
     horizon), and
   - tick up to **20 satellites** from the AMSAT list (a live counter enforces
     the cap).
3. Tap **Finish setup** (or hold **KEY1**) and StickSat downloads the SatNOGS
   transponder list for **every** satellite you selected and caches it to flash
   (you'll see per-satellite progress on the device), then drops into the
   Next-Passes screen and starts tracking. Because the GP elements and all
   transponder data are now on flash, **the tracker keeps working fully offline
   / out of WiFi range** — the Track screen's Doppler and transponder values
   come from the cache, no network needed.

## Normal power-on (after first setup)

StickSat **never returns to initial setup on its own**. On every power-on it
keeps the saved WiFi credentials and tries to connect to that network:

- **Connected** — the clock NTP-syncs, fresh data can be fetched, and it lands
  on the Next Passes screen.
- **Not connected** — it still goes straight to the main screen and runs from the
  cached GP / transponder data and the deep-sleep-preserved clock. A red **`NW`**
  ("No WiFi") tag shows in the header on every screen as the warning.

To change the network, location, or satellite list at any time, **long-press
KEY2** on any screen. StickSat reconnects to WiFi (or re-opens the captive
portal if the saved network isn't reachable — e.g. you've moved), serves the
setup page again, and on Finish re-caches transponders for the updated list.
Location and satellite selection are always preserved on flash; a factory reset
(erase flash) or re-flash is the only thing that returns to first-boot setup.

---

## Optional GPS (M5Stack Unit GPS v1.1)

Plug an **M5Stack Unit GPS v1.1** into the Grove port and StickSat uses it
automatically — no setup needed. At boot it opens the Grove UART (G33 RX / G32
TX, 115200 NMEA) and, once the unit gets a fix, sets the **clock** from GPS UTC
and the **location** from the GPS position (then saves the location to flash so
it persists without GPS). A green **`G`** appears in the header while a fix is
active. This is the easiest way to run fully offline: no WiFi, no manual clock
entry. GPS is entirely optional — if nothing is connected, nothing happens and
no error is shown.

---

## Setting the clock without WiFi or GPS

Long-press **KEY2** → the setup chooser → **KEY2** again opens a button-only UTC
clock editor:

- **KEY2** increments the highlighted field (hold to auto-repeat, accelerating).
- **KEY1** moves to the next field (Year → Month → Day → Hour → Min → Sec).
- **hold KEY1** saves and exits.

The clock is also **preserved across deep sleep**: StickSat stashes a time
anchor in RTC memory before sleeping and restores it on wake, so it stays usable
offline through sleep/wake cycles. (The ESP32's deep-sleep clock drifts on its
internal oscillator — fine for the short sleeps StickSat uses between passes;
for long offline stretches, a GPS or WiFi re-sync keeps it honest.)

---

## Build & flash (PlatformIO)

```
pio run                 # build
pio run -t upload       # flash
pio device monitor      # 115200 baud log
```

The `m5stickc-plus` env pins **M5Unified**,
**ArduinoJson** v7, and the Hopperpop **SGP4** library, on
`board = m5stick-c` (the PlatformIO id for the StickC family) with the bundled
4 MB partition table (`partitions_4mb.csv`, single app + a large LittleFS data
region). PSRAM is **not** enabled.

### Memory note (no PSRAM)

The PICO-D4 has no PSRAM, so the ~64 KB full-frame off-screen sprite is
allocated from internal SRAM. If it can't be allocated (heap fragmented by
WiFi/TLS), StickSat automatically falls back to drawing directly to the panel —
slightly more flicker, but fully functional. The satellite catalog held in RAM
is capped (`MAX_SATS` in `config.h`), and the GP download is streamed to flash
rather than buffered. When re-opening setup via KEY2, the sprite is temporarily
freed so the web server and TLS have the headroom they need.

### Arduino IDE

A single-file `StickSat.ino` is generated from the modular sources for the
Arduino IDE. Install **M5Unified** (pulls in M5GFX), **ArduinoJson** (v7), and
the Hopperpop **Sgp4** library (Add .ZIP Library from
<https://github.com/Hopperpop/Sgp4-Library>). Board: **M5StickC-Plus** (ESP32
PICO, 4 MB flash, a partition scheme with a SPIFFS/LittleFS region). Do **not**
enable PSRAM (the PICO-D4 has none). The optional GPS needs **no extra library** —
NMEA is parsed in `gps.cpp`.

---

## Data sources

* Orbital data: **AMSAT GP/OMM JSON** —
  `https://newark192.amsat.org/gpdata/current/daily-bulletin.json`, streamed
  straight to flash and parsed one element set at a time.
* Transponder frequencies: **SatNOGS DB** —
  `https://db.satnogs.org/api/transmitters/`, cached per satellite.

TLS uses `setInsecure()` (no certificate validation) — fine for public data;
pin a CA root if you need it.

Optional hardware: **M5Stack Unit GPS v1.1** (ATGM336H/AT6668, NMEA 0183 over
UART at 115200) on the Grove port — ESP32 RX=G33, TX=G32.

---

## File map

```
platformio.ini      board, libs, build flags (M5StickC Plus / ESP32-PICO, 4MB, no PSRAM)
partitions_4mb.csv  4 MB partition table: single app + large LittleFS region
StickSat.ino        generated single-file Arduino build
main.cpp            entry point: boot -> (first-run portal/setup) -> App
config.h            URLs, button + GPS pins, limits, file paths
storage.{h,cpp}     LittleFS filesystem layer
settings.{h,cpp}    persisted config (WiFi, location, min elev, alarm)
satdb.{h,cpp}       GP/OMM store + TLE rebuild + streaming parse + transponders
net.{h,cpp}         WiFi, NTP, HTTPS GET, GP stream-to-file, SatNOGS fetch
location.{h,cpp}    manual / grid position, Maidenhead conversion
predict.{h,cpp}     SGP4 wrapper: look angles, passes, Doppler, Sun/eclipse, polar
gps.{h,cpp}         optional Grove GPS (Unit GPS v1.1): NMEA time + location
favs.{h,cpp}        the user's selected satellites (up to 20)
buttons.{h,cpp}     KEY1 / KEY2 over M5Unified BtnA / BtnB (short + long press), wake pin
portal.{h,cpp}      captive WiFi portal + location/satellite setup web server
app.{h,cpp}         4-screen UI state machine, GPS, AOS alarm, deep sleep, clock entry
```

## Credits

* SGP4 propagation: [Hopperpop/Sgp4-Library](https://github.com/Hopperpop/Sgp4-Library).
* GP data: [AMSAT](https://www.amsat.org/). Transponders: [SatNOGS DB](https://db.satnogs.org/).
* Doppler approach and original tracker: **CardSat**.

Built for amateur-radio use; respect your local licensing and band plans.
