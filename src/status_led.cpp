#include "status_led.hpp"

#include "esp_check.h"
#include "gflib/link.hpp"

namespace led {

namespace {

const char* kTag = "led";

// 0.1us ticks
constexpr uint32_t kRmtHz = 10 * 1000 * 1000;
constexpr uint16_t kShortTicks = 3;
constexpr uint16_t kLongTicks = 9;

// out of 255
constexpr uint8_t kMax = 100;

//refresh frequency
constexpr uint32_t kRefreshMs = 50;

//how long to hold error statuses
constexpr uint32_t kHoldMs = 1000;

constexpr uint32_t kFlashPeriodMs = 500;
constexpr uint32_t kQuickFlashPeriodMs = 150;
constexpr uint32_t kPulsePeriodMs = 2000;

struct Pattern {
    uint8_t r, g, b;
    uint32_t periodMs;
};

// prioritizes higher priority status
constexpr Pattern kPatterns[] = {
    {kMax, 0, 0, kFlashPeriodMs},               // red flashing
    {kMax, kMax / 4, 0, kFlashPeriodMs},        // orange flashing
    {kMax, kMax / 2, 0, kFlashPeriodMs},        // yellow flashing
    {0, 0, kMax, kFlashPeriodMs},               // blue flashing
    {kMax, 0, kMax, kQuickFlashPeriodMs},       // magenta quick flashing
    {kMax, kMax, kMax, kFlashPeriodMs},         // white flashing
};

rmt_symbol_word_t bitSymbol(uint16_t highTicks, uint16_t lowTicks) {
    rmt_symbol_word_t s = {};
    s.level0 = 1;
    s.duration0 = highTicks;
    s.level1 = 0;
    s.duration1 = lowTicks;
    return s;
}

}  // namespace

esp_err_t StatusLed::begin(int pin) {
    rmt_tx_channel_config_t tc = {};
    tc.gpio_num = static_cast<gpio_num_t>(pin);
    tc.clk_src = RMT_CLK_SRC_DEFAULT;
    tc.resolution_hz = kRmtHz;
    tc.mem_block_symbols = 48;
    tc.trans_queue_depth = 1;
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tc, &chan_), kTag, "channel");

    rmt_bytes_encoder_config_t ec = {};
    ec.bit0 = bitSymbol(kShortTicks, kLongTicks);
    ec.bit1 = bitSymbol(kLongTicks, kShortTicks);
    ec.flags.msb_first = 1;
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&ec, &enc_), kTag, "encoder");

    ESP_RETURN_ON_ERROR(rmt_enable(chan_), kTag, "enable");
    show(kMax, kMax, kMax);
    return ESP_OK;
}

void StatusLed::show(uint8_t r, uint8_t g, uint8_t b) {
    if (enc_ == nullptr) return;
    
    rmt_tx_wait_all_done(chan_, -1);
    grb_[0] = g;
    grb_[1] = r;
    grb_[2] = b;

    rmt_transmit_config_t xc = {};
    rmt_transmit(chan_, enc_, grb_, sizeof(grb_), &xc);
}

void StatusLed::update(uint32_t nowMs, const Inputs& in) {
    const bool now[kStates] = {in.sensorFault, in.tofFault, in.linkDown, in.brainDisabled, in.poseResetPending, in.mclNotConverged};
    for (int i = 0; i < kStates; ++i) {
        if (now[i]) {
            seenMs_[i] = nowMs;
            seen_[i] = true;
        }
    }

    if (enc_ == nullptr || gflib::linkElapsedMs(nowMs, lastRefreshMs_) < kRefreshMs) return;
    lastRefreshMs_ = nowMs;

    for (int i = 0; i < kStates; ++i) {
        if (!seen_[i] || gflib::linkElapsedMs(nowMs, seenMs_[i]) >= kHoldMs) continue;

        const Pattern& p = kPatterns[i];
        if (nowMs % p.periodMs < p.periodMs / 2) {
            show(p.r, p.g, p.b);
        } else {
            show(0, 0, 0);
        }
        return;
    }

    //green led
    // Always changing and never goes to 0
    const uint32_t t = nowMs % kPulsePeriodMs;
    const uint32_t half = kPulsePeriodMs / 2;
    const uint32_t ramp = (t < half) ? t : kPulsePeriodMs - t;
    show(0, static_cast<uint8_t>(2 + ramp * (kMax - 2) / half), 0);
}

}  // namespace led
