// ===========================================================================
//  buttons.cpp
// ===========================================================================
#include "buttons.h"
#include "config.h"
#include <M5Unified.h>

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
