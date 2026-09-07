#pragma once
#include <cstddef>
#include <cstdint>

// TMF8821 RAM image loaded on every cold start (AN001015 2.2.1).
// Source: SparkFun Qwiic TMF882X `tof_bin_image.c`, one 0x9AC-byte block.

namespace tof {

extern const uint8_t kFirmwareImage[];
extern const size_t  kFirmwareImageLen;
inline constexpr uint16_t kFirmwareLoadAddr = 0x0000;

}  // namespace tof
