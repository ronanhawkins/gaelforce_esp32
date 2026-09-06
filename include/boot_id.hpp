#pragma once
#include <cstdint>

namespace bootid {

// Reads, increments and commits a counter in NVS, returning the new value.
// Used to tell brain that esp has restarted.
uint16_t next();

}  // namespace bootid
