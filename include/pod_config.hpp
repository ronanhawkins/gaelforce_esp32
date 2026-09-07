#pragma once
#include "gflib/posesource.hpp"
#include "gflib/util.hpp"
#include "gflib/link.hpp"
#include "gflib/mcl.hpp"

// Every number that describes this particular robot lives in this file.
// Nothing below it knows a pin number or a wheel diameter, so retuning the
// machine never means reading the driver code.

namespace cfg {

using gflib::real;
using gflib::operator""_r;

// pins
//
// Unavailable on this board and deliberately absent below: 0/45/46 strapping,
// 19/20 USB, 26-32 SPI flash, 33-37 the octal PSRAM die -- the N8R8 module
// bonds those whether or not the software enables PSRAM -- and 48 the RGB LED.
// 43/44 are the console UART. That leaves 1-18, 21, 38-42.

// Quadrature pods, through the SN74LVC244A. The 244 drives actively, so the
// PCNT channels' default pull-ups are along for the ride and harmless.
constexpr int kVertEncAPin  = 4;
constexpr int kVertEncBPin  = 5;
constexpr int kHorizEncAPin = 6;
constexpr int kHorizEncBPin = 7;

// BNO085 UART-RVC. RX only: RVC is one-way and the part accepts nothing back.
// The P0 jumper on the breakout must be bridged or it comes up in I2C mode
// and this pin stays silent forever -- indistinguishable from a broken wire.
constexpr int kImuRxPin = 8;

// MAX3485. DE and RE are tied together and driven as UART RTS, which lets
// uart_set_mode(UART_MODE_RS485_HALF_DUPLEX) handle the turnaround in
// hardware. Doing it from software is the classic RS-485 bug: release a bit
// early and the last byte is truncated, a bit late and it stamps on the reply.
constexpr int kRs485TxPin = 17;
constexpr int kRs485RxPin = 18;
constexpr int kRs485DePin = 16;

// Stage B. Listed here so the pin budget is decided once, not twice.
constexpr int kI2cSdaPin = 9;
constexpr int kI2cSclPin = 10;
constexpr int kTofEnPins[4] = {11, 12, 13, 14};

// encoders

// AMT102-V, switch-selected to 2048 PPR. Full quadrature counts all four
// edges of each cycle, so a revolution is 4x the PPR. Getting this wrong
// scales every distance by exactly 4, which reads as a plausible calibration
// error rather than a bug.
constexpr real kCountsPerRev = 8192.0_r;

// PLACEHOLDER, not a measurement. Nothing downstream is trustworthy until
// this is the real wheel.
constexpr real kTrackingWheelDiaIn = 2.0_r;

constexpr bool kVertReversed  = false;
constexpr bool kHorizReversed = false;

// Well under the shortest real edge: at the placeholder 2in wheel, 5ft/s is
// an edge every ~13us. Also under the peripheral's own ~12.7us ceiling.
constexpr uint32_t kEncoderGlitchFilterNs = 1000;

// The hardware counter is 16-bit signed. The driver accumulates past these
// only if watch points sit exactly on them -- see PcntEncoder::begin.
constexpr int kPcntHighLimit =  32000;
constexpr int kPcntLowLimit  = -32000;

// odom geometry

// Confirmed: the pods are mounted as a diamond, not forward/sideways.
constexpr real kPodAngleDeg = 45.0_r;

// PLACEHOLDERS. Signed perpendicular distances from the tracking centre --
// the sign matters as much as the magnitude, because a sign error injects
// rotation into the pose as though it were travel.
constexpr real kVertOffsetIn  = 3.0_r;
constexpr real kHorizOffsetIn = 3.0_r;

inline gflib::OdomSourceConfig makeOdom() {
    gflib::OdomSourceConfig c;

    c.odom.podAngleDeg = kPodAngleDeg;
    c.odom.vertOffsetInches = kVertOffsetIn;
    c.odom.horizOffsetInches = kHorizOffsetIn;

    const real inchesPerCount = gflib::kPi * kTrackingWheelDiaIn / kCountsPerRev;
    c.odom.vertInchesPerCount = inchesPerCount;
    c.odom.horizInchesPerCount = inchesPerCount;

    return c;
}

// loop and link

constexpr uint32_t kLoopPeriodMs = 10;

// Spent draining the RX line after the pose frame goes out
constexpr int64_t kRxWindowUs = 8000;

constexpr int64_t kRxWindowIdleUs = 1000;

// Three missed replies. The Brain sends one per pose frame, so this is a link
// that has actually stopped rather than one that dropped a frame.
constexpr uint32_t kBrainStatusTimeoutMs = 30;

// IMU timeout, if longer than this then likely dead connection
constexpr uint32_t kImuFrameTimeoutMs = 50;

// 8N1 on the wire is one start bit, eight data, one stop.
constexpr real kBitsPerSerialByte = 10.0_r;

// Air time is exactly computable and must be subtracted from a round trip
// before halving it: the pose frame is 56 bytes and the status reply 23, so
// splitting the raw figure evenly biases the result by most of a millisecond.
inline real airTimeMs(size_t bytes) {
    return (static_cast<real>(bytes) * kBitsPerSerialByte * 1000.0_r)
           / static_cast<real>(gflib::kLinkBaud);
}

// ToF sensors

// Indices must match SensorReading::mountIndex.
enum TofIndex : int { kTofFront = 0, kTofRight = 1, kTofRear = 2, kTofLeft = 3 };
constexpr int kTofCount = 4;

// All TMF8821s boot at volatile address 0x41.
constexpr uint8_t kTofBootAddr = 0x41;

constexpr uint8_t kTofAddrs[kTofCount] = {0x42, 0x43, 0x44, 0x45};

// Enumeration runs at 100kHz.
constexpr uint32_t kTofEnumHz = 100000;

// Runtime speed is configured per device.
constexpr uint32_t kTofRunHz = 400000;

// Deadline-poll timeouts.
constexpr uint32_t kTofEnableTimeoutMs = 50;
constexpr uint32_t kTofCommandTimeoutMs = 100;
constexpr uint32_t kTofAppStartTimeoutMs = 500;

// Staggered 40 ms captures yield one block read per tick.
constexpr uint16_t kTofPeriodMs = 40;
constexpr uint32_t kTofStaggerMs = 10;

// spad_map_id 7, 4x4 normal, 41x52 degrees. The 4x4 maps are TMF8821-only.
constexpr uint8_t kTofSpadMap = 7;
constexpr uint16_t kTofKiloIterations = 537;

// ToF geometry and preprocessing

// Boards are rotated so the 41° FoV axis is vertical.
constexpr int kTofZones[4] = {2, 6, 11, 15};

// Zone offsets used to project readings onto the boresight.
constexpr real kTofZoneOffAxisDeg[4] = {-19.5_r, -6.5_r, 6.5_r, 19.5_r};

// Trust radius, not the detection limit.
constexpr real kTofTrustRadiusIn = 85.0_r;

// Reject grazing returns beyond this incidence.
constexpr real kTofMaxIncidenceDeg = 60.0_r;

// Maximum deviation from the projected median.
constexpr real kTofZoneDispersionIn = 4.0_r;

// Provisional motion gates; tune from Reject::Motion counts.
constexpr real kTofMaxYawDegPerSec = 180.0_r;
constexpr real kTofMaxDriveVolts = 10.0_r;

// PLACEHOLDERS: signed tracking-center-to-lens offsets.
constexpr real kTofFrontOffsetIn = 8.0_r;
constexpr real kTofRightOffsetIn = 8.0_r;
constexpr real kTofRearOffsetIn  = 8.0_r;
constexpr real kTofLeftOffsetIn  = 8.0_r;

// Bearings are fixed. Compass: +Y forward, +X right, 0 ahead, CW+.
constexpr gflib::SensorMount kTofMounts[kTofCount] = {
    {             0.0_r,  kTofFrontOffsetIn,   0.0_r},  // kTofFront
    { kTofRightOffsetIn,              0.0_r,  90.0_r},  // kTofRight
    {             0.0_r, -kTofRearOffsetIn,  180.0_r},  // kTofRear
    {-kTofLeftOffsetIn,               0.0_r, 270.0_r},  // kTofLeft
};

// MCL

inline gflib::MclConfig makeMcl() {
    gflib::MclConfig c;

    c.particleCount = 200;

    // PLACEHOLDER: interior half-span to the inner wall face.
    c.fieldHalfWidthInches  = 72.0_r;
    c.fieldHalfHeightInches = 72.0_r;

    c.sensorSigmaInches = 1.5_r;

    // Prediction clamp above the field diagonal; not a trust radius.
    c.sensorMaxRangeInches = 250.0_r;

    // Tighter short gate rejects nearer robots and game objects.
    c.gateShortInches = 8.0_r;
    c.gateLongInches  = 12.0_r;

    // Allow contradictory readings while confidence is low.
    c.gateMinConfidence = 0.3_r;

    return c;
}

// Tight reseeds avoid field-symmetry ambiguity.
constexpr real kMclSeedPosSigmaIn = 4.0_r;
constexpr real kMclSeedHeadingSigmaDeg = 5.0_r;

// Rate-limit recovery from latched divergence using dead reckoning.
constexpr uint32_t kMclReseedMinIntervalMs = 500;

}  // namespace cfg
