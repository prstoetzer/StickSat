// ===========================================================================
//  favs.cpp
// ===========================================================================
#include "favs.h"
#include "storage.h"
#include <LittleFS.h>

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
