#pragma once
#include "gflib/posesource.hpp"
#include "gflib/util.hpp"
#include "gflib/link.hpp"
#include "gflib/mcl.hpp"
#include "tmf8821.hpp"

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

// Quadrature pods, through the SN74LVC245A. The 245 drives actively, so the
// PCNT channels' default pull-ups are along for the ride and harmless.
constexpr int kVertEncAPin  = 1;
constexpr int kVertEncBPin  = 2;
constexpr int kHorizEncAPin = 4;
constexpr int kHorizEncBPin = 5;

// BNO085 UART-RVC. RX only: RVC is one-way and the part accepts nothing back.
// The P0 jumper on the breakout must be bridged or it comes up in I2C mode
// and this pin stays silent forever -- indistinguishable from a broken wire.
constexpr int kImuRxPin = 8;

// SP3485. DE and RE are tied together and driven as UART RTS, which lets
// uart_set_mode(UART_MODE_RS485_HALF_DUPLEX) handle the turnaround in
// hardware. Doing it from software is the classic RS-485 bug: release a bit
// early and the last byte is truncated, a bit late and it stamps on the reply.
constexpr int kRs485TxPin = 17;
constexpr int kRs485RxPin = 18;
constexpr int kRs485DePin = 21;

// Stage B. Listed here so the pin budget is decided once, not twice.
constexpr int kI2cSdaPin = 15;
constexpr int kI2cSclPin = 16;
// Harness order, not numeric order
// The array is indexed by TofIndex alongside kTofAddrs and kTofMounts
// the wiring is absorbed here rather than by renumbering the enum.
constexpr int kTofEnPins[4] = {11, 12, 14, 13};

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

// 400k keeps the 48-byte read near 1ms; 100k overran kLoopPeriodMs. Needs the
// bus pull-ups at 1.5k or below. Watch busErrors() after changing this.
constexpr uint32_t kTofRunHz = 400000;

// Deadline-poll timeouts.
constexpr uint32_t kTofEnableTimeoutMs = 50;
constexpr uint32_t kTofCommandTimeoutMs = 100;
constexpr uint32_t kTofAppStartTimeoutMs = 500;

//staggered 70ms captures
constexpr uint16_t kTofPeriodMs = 70;
constexpr uint32_t kTofStaggerMs = 17;

// User-defined, single capture: 2 SPADs tall and centred. The stock 4x4 row sat
// off-axis and floor-saturated at ~64in. Reverting means restoring the angles.
constexpr uint8_t kTofSpadMap = 14;

// A zone now has 4 SPADs where the stock map had ~11, so this buys most of the
// light back. Above ~1150k the ranging period overruns kTofPeriodMs.
constexpr uint16_t kTofKiloIterations = 1000;

// Registers 0x24-0x90, contiguous. Generated, never hand-edited: the enable and
// TDC fields are bit-packed and one wrong byte silently remaps a zone.
constexpr uint8_t kTofSpadMaskBlob[] = {
    0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xC0, 0x03, 0x00, 0xCC,
    0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x02,
    0x08
};

static_assert(kTofSpadMap != 14 || sizeof(kTofSpadMaskBlob) == 0x90 - 0x24 + 1,
              "spad_map_id 14 needs all 109 SPAD page bytes from the ams tool");

// ToF geometry and preprocessing

// assumes a zone reports in the slot matching its TDC channel.
// One capture, so unlike the stock map these four are simultaneous.
constexpr int kTofZones[4] = {2, 4, 6, 8};

static_assert(kTofZones[0] <= tof::kZonesRead && kTofZones[1] <= tof::kZonesRead &&
              kTofZones[2] <= tof::kZonesRead && kTofZones[3] <= tof::kZonesRead,
              "a zone past kZonesRead is never fetched and reads as no-target");

// DERIVED from the SPAD pitch, not measured. MUST change with kTofSpadMap.
constexpr real kTofZoneOffAxisDeg[4] = {-16.22_r, -5.54_r, 5.54_r, 16.22_r};

// DERIVED, not measured: where the 4.81 deg band catches the floor at a 5.75in
// mount. Small-angle geometry, so the real knee could be 20in either side.
constexpr real kTofTrustRadiusIn = 137.0_r;

// Within this of the knee, a floor return and a wall read the same.
constexpr real kTofSaturationMarginIn = 6.0_r;

// Fires only when the estimate disagrees with the measurement; the spread gate
// is always tighter. Loose, so a bad estimate cannot starve its own recovery.
constexpr real kTofMaxIncidenceDeg = 60.0_r;

// Oblique-wall spread scales with range, so a fixed inch gate quietly tightened
// as you backed away. As a fraction it admits ~20 deg at any range.
constexpr real kTofZoneSpreadFrac = 0.15_r;

// Keeps sensorSigmaInches noise from tripping the fraction at close range.
constexpr real kTofZoneSpreadFloorIn = 2.5_r;

// Half the ranging period plus a tick to notice it. The record says where the
// robot WAS; uncorrected it drags the cloud back along the direction of travel.
constexpr real kTofLagMs = 35.0_r;

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

// gflib caps this at kMclMaxParticles and init() CLAMPS SILENTLY, hence the
// assert. More particles buy relocalisation, not converged accuracy.
constexpr int kMclParticles = 300;

static_assert(kMclParticles <= gflib::kMclMaxParticles,
              "particleCount above kMclMaxParticles is clamped without a word");

inline gflib::MclConfig makeMcl() {
    gflib::MclConfig c;

    c.particleCount = kMclParticles;

    // PLACEHOLDER: interior half-span to the inner wall face.
    c.fieldHalfWidthInches  = 72.0_r;
    c.fieldHalfHeightInches = 72.0_r;

    // Restated at the gflib default so both floors sit together. This is the
    // diffusion that keeps a parked cloud honest.
    c.transNoiseFloorInches = 0.05_r;

    // One shared IMU, arriving as a delta, and range barely sees it: 0.011
    // in/deg axis-aligned. Non-zero anyway, because position coupling still
    // prunes a wrong heading and zero would make an IMU error silent.
    c.headingNoisePerDeg = 0.005_r;
    c.headingNoisePerInch = 0.005_r;
    c.headingNoiseFloorDeg = 0.005_r;

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
