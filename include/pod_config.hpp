#pragma once
#include "gflib/posesource.hpp"
#include "gflib/util.hpp"
#include "gflib/link.hpp"

// Everything describing the physical

namespace cfg {

using gflib::real;
using gflib::operator""_r;

// pins

// Quadrature pods, through the SN74LVC244A.
constexpr int kVertEncAPin  = 4;
constexpr int kVertEncBPin  = 5;
constexpr int kHorizEncAPin = 6;
constexpr int kHorizEncBPin = 7;

// BNO085 UART-RVC. RX only
// The P0 jumper on the breakout must be bridged or it comes up in I2C mode
constexpr int kImuRxPin = 8;

// MAX3485. DE and RE are tied together and driven as UART RTS, which lets
// uart_set_mode(UART_MODE_RS485_HALF_DUPLEX) handle the turnaround in hardware.
constexpr int kRs485TxPin = 17;
constexpr int kRs485RxPin = 18;
constexpr int kRs485DePin = 16;

// I2C and ToF Pins for Monte Carlo Localization
constexpr int kI2cSdaPin = 9;
constexpr int kI2cSclPin = 10;
constexpr int kTofEnPins[4] = {11, 12, 13, 14};

// encoders

// AMT102-V, switch-selected to 2048 PPR. Full quadrature counts all four
// edges of each cycle, so a revolution is 4x the PPR.
constexpr real kCountsPerRev = 8192.0_r;

// PLACEHOLDER ***
constexpr real kTrackingWheelDiaIn = 2.0_r;

constexpr bool kVertReversed  = false;
constexpr bool kHorizReversed = false;

// PCNT glitch filter
constexpr uint32_t kEncoderGlitchFilterNs = 1000;

// The hardware counter is 16-bit signed. The driver accumulates past these
// only if watch points sit exactly on them
constexpr int kPcntHighLimit =  32000;
constexpr int kPcntLowLimit  = -32000;

// odom geometry

// Angle between odom pods and front
constexpr real kPodAngleDeg = 45.0_r;

// PLACEHOLDERS *** Signed perpendicular distances from the tracking centre sign matters
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

// period of link
constexpr uint32_t kLoopPeriodMs = 10;

// Spent draining the RX line after the pose frame goes out
constexpr int64_t kRxWindowUs = 8000;

// Three missed replies. The Brain sends one per pose frame, so this is a link
// that has actually stopped rather than one that dropped a frame.
constexpr uint32_t kBrainStatusTimeoutMs = 30;

// IMU timeout, likely dead wiring/sensor if longer than this
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

}  // namespace cfg
