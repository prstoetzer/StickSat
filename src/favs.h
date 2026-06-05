#pragma once
// ===========================================================================
//  favs.h  -  the user's selected satellites (NORAD ids), persisted to flash
// ===========================================================================
//  Up to MAX_FAVS satellites, one NORAD id per line in FILE_FAVS. Shared by the
//  setup web server (which writes the selection) and the app (which schedules
//  passes for them and scrolls through them with KEY2).
#include <Arduino.h>
#include "config.h"

namespace Favs {
  int  load(uint32_t* out, int maxN);              // -> count
  bool save(const uint32_t* ids, int n);
  bool contains(const uint32_t* ids, int n, uint32_t norad);
}
