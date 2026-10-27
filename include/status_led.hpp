#pragma once
#include <cstdint>

#include "driver/rmt_tx.h"
#include "esp_err.h"

namespace led {

// Highest priority first
struct Inputs {
    bool sensorFault = false;   // IMU or either encoder pod
    bool tofFault = false;
    bool linkDown = false;
    bool brainDisabled = false;
    bool poseResetPending = false;
    bool mclNotConverged = false;
};

class StatusLed {
    public:
        // Lights solid white, holds until the first update().
        esp_err_t begin(int pin);

        // Call every tick to check for faults. pixel rewritten only every kRefreshMs.
        void update(uint32_t nowMs, const Inputs& in);

    private:
        static constexpr int kStates = 6;

        void show(uint8_t r, uint8_t g, uint8_t b);

        rmt_channel_handle_t chan_ = nullptr;
        rmt_encoder_handle_t enc_ = nullptr;

        // reads after rmt_transmit() returns.
        uint8_t grb_[3] = {};

        uint32_t lastRefreshMs_ = 0;

        // Last tick each condition was seen
        uint32_t seenMs_[kStates] = {};
        bool seen_[kStates] = {};
};

}  // namespace led
