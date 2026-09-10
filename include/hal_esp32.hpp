#pragma once
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "gflib/hal.hpp"

// The pod's side of gflib's HAL. Every class here is a transport, none of
// them know what a pose is.
//
// Each sensor separates ADVANCING the hardware from READING it. sample()
// latches; the getters return the latch. The loop calls sample() once at the
// top of a tick, so OdomPoseSource and Mcl::predict are looking at the same
// encoder read rather than two reads a few hundred microseconds apart. That
// drift would be small, slow, and almost impossible to attribute after the fact.

namespace hal {

class EspClock : public gflib::IClock {
    public:
        // esp_timer is monotonic from boot. The truncation to 32 bits wraps
        // at ~49.7 days, which is exactly what linkElapsedMs assumes.
        uint32_t millisNow() const override;

        void sleepMs(uint32_t ms) override;
};

// One PCNT unit per pod, both channels, all four edges.
class PcntEncoder : public gflib::IEncoder {
    public:
        PcntEncoder(int pinA, int pinB, bool reversed);

        esp_err_t begin();

        // Advances the latch. Everything else is a read of it.
        void sample();

        double getCounts() const override { return counts_; }

        // The tick's delta, in counts. Also what Mcl::predict wants
        double deltaCounts() const { return counts_ - prevCounts_; }

    private:
        pcnt_unit_handle_t unit_ = nullptr;
        pcnt_channel_handle_t chanA_ = nullptr;
        pcnt_channel_handle_t chanB_ = nullptr;

        int pinA_, pinB_;
        bool reversed_;

        double counts_ = 0.0;
        double prevCounts_ = 0.0;
};

// BNO085 in UART-RVC mode: 115200 8N1, one unsolicited 19-byte frame at
// 100Hz, receive only.
//
//   [0][1]   0xAA 0xAA
//   [2]      index, rolls 0..255
//   [3][4]   yaw, int16 LE, 0.01 deg, counter-clockwise positive
//   [5..17]  pitch, roll, accel, reserved -- unused here
//   [18]     checksum: sum of [2..17] truncated to 8 bits
class RvcImu : public gflib::IImu {
    public:
        RvcImu(uart_port_t port, int rxPin);

        esp_err_t begin();

        // Drains the driver's ring buffer and parses whatever completed.
        //
        // The unwrap below is per-parsed-frame
        void sample();

        // Unwrapped, clockwise positive, 0 = +Y, as IImu requires.
        double getHeadingDeg() const override { return headingDeg_; }

        uint32_t frameCount() const { return frames_; }
        uint32_t checksumErrors() const { return badChecksums_; }

        // Frames are still arriving. Flat frameCount is dead sensor or dead
        // wiring, never a stationary robot. RVC streams unprompted.
        bool ok(uint32_t nowMs) const;

    private:
        void feed(const uint8_t* data, size_t len);
        void onFrame(const uint8_t* f);

        uart_port_t port_;
        int rxPin_;

        uint8_t buf_[19] = {};
        uint8_t len_ = 0;

        // last wrapped RVC yaw, CCW+
        double rawYawDeg_ = 0.0;
        double revolutions_ = 0.0;
        // latched, CW+
        double headingDeg_ = 0.0;
        bool seeded_ = false;

        uint32_t frames_ = 0;
        uint32_t badChecksums_ = 0;
        uint32_t lastFrameMs_ = 0;
};

// SP3485 half duplex, with the driver enable driven as hardware RTS.
class Rs485Stream : public gflib::IByteStream {
    public:
        Rs485Stream(uart_port_t port, int txPin, int rxPin, int dePin);

        esp_err_t begin();

        size_t read(uint8_t* dst, size_t cap) override;

        // No partial writes
        size_t write(const uint8_t* src, size_t len) override;

        // Blocks up to timeoutUs for the first byte. Returns 0 on timeout.
        size_t readBlocking(uint8_t* dst, size_t cap, int64_t timeoutUs);

    private:
        uart_port_t port_;
        int txPin_, rxPin_, dePin_;
};

}  // namespace hal
