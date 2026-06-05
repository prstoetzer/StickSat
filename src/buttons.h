#pragma once
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
#include <Arduino.h>

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
