#pragma once
// ===========================================================================
//  storage.h -- filesystem abstraction (internal LittleFS)
// ===========================================================================
//  StickSat persists everything to internal flash via LittleFS. (CardSat also
//  carried a microSD fallback for launcher use; the StickS3 build flashes its
//  own partition table with a LittleFS data region, so the SD path is dropped.)
#include <FS.h>

namespace Store {
  bool    begin();          // mount LittleFS (format on first-boot failure)
  fs::FS& fs();             // the active filesystem
  bool    ready();          // true if the filesystem mounted
  bool    format();         // wipe LittleFS (factory reset)
}
