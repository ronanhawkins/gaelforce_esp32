#include "hal_esp32.hpp"

#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pod_config.hpp"

namespace hal {

namespace {
const char* kTag = "hal";
}

// clock

uint32_t EspClock::millisNow() const {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

void EspClock::sleepMs(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// encoders

PcntEncoder::PcntEncoder(int pinA, int pinB, bool reversed)
    : pinA_(pinA), pinB_(pinB), reversed_(reversed) {}

esp_err_t PcntEncoder::begin() {
    pcnt_unit_config_t unitCfg = {};
    unitCfg.low_limit = cfg::kPcntLowLimit;
    unitCfg.high_limit = cfg::kPcntHighLimit;
    unitCfg.flags.accum_count = 1;
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unitCfg, &unit_), kTag, "pcnt_new_unit");

    pcnt_glitch_filter_config_t filterCfg = {};
    filterCfg.max_glitch_ns = cfg::kEncoderGlitchFilterNs;
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(unit_, &filterCfg), kTag, "glitch filter");

    // accum_count is silent about its own preconditions: without a watch
    // point sitting exactly on each limit there is no interrupt to accumulate
    // in, and the counter simply wraps.
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(unit_, cfg::kPcntHighLimit), kTag,
                        "watch high (accum_count does nothing without it)");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(unit_, cfg::kPcntLowLimit), kTag,
                        "watch low (accum_count does nothing without it)");

    // Full quadrature. Each channel takes one line as its edge signal and the
    // other as the level that decides direction, so all four edges of a cycle
    // count and the pair is self-cancelling under dither.
    pcnt_chan_config_t chanACfg = {};
    chanACfg.edge_gpio_num = pinA_;
    chanACfg.level_gpio_num = pinB_;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unit_, &chanACfg, &chanA_), kTag, "chan a");

    pcnt_chan_config_t chanBCfg = {};
    chanBCfg.edge_gpio_num = pinB_;
    chanBCfg.level_gpio_num = pinA_;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unit_, &chanBCfg, &chanB_), kTag, "chan b");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chanA_,
                            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                            PCNT_CHANNEL_EDGE_ACTION_INCREASE), kTag, "a edge");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chanA_,
                            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                            PCNT_CHANNEL_LEVEL_ACTION_INVERSE), kTag, "a level");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chanB_,
                            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                            PCNT_CHANNEL_EDGE_ACTION_DECREASE), kTag, "b edge");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chanB_,
                            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                            PCNT_CHANNEL_LEVEL_ACTION_INVERSE), kTag, "b level");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unit_), kTag, "enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unit_), kTag, "clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unit_), kTag, "start");

    return ESP_OK;
}

void PcntEncoder::sample() {
    // Advanced before the read can fail, so a failed read leaves the latch and
    // the baseline equal and deltaCounts() reports zero.
    prevCounts_ = counts_;

    int raw = 0;
    if (pcnt_unit_get_count(unit_, &raw) != ESP_OK) return;

    counts_ = reversed_ ? -static_cast<double>(raw) : static_cast<double>(raw);
}

// imu

namespace {
constexpr uint8_t kRvcHeader = 0xAA;
constexpr uint8_t kRvcFrameLen = 19;
constexpr double kRvcYawLsbDeg = 0.01;

// Two ticks of frames plus slack.
constexpr int kRvcRxBufBytes = 512;
}  // namespace

RvcImu::RvcImu(uart_port_t port, int rxPin) : port_(port), rxPin_(rxPin) {}

esp_err_t RvcImu::begin() {
    uart_config_t c = {};
    c.baud_rate = 115200;
    c.data_bits = UART_DATA_8_BITS;
    c.parity = UART_PARITY_DISABLE;
    c.stop_bits = UART_STOP_BITS_1;
    c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    c.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(uart_driver_install(port_, kRvcRxBufBytes, 0, 0, nullptr, 0),
                        kTag, "rvc driver");
    ESP_RETURN_ON_ERROR(uart_param_config(port_, &c), kTag, "rvc config");
    ESP_RETURN_ON_ERROR(uart_set_pin(port_, UART_PIN_NO_CHANGE, rxPin_,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        kTag, "rvc pins");
    return ESP_OK;
}

void RvcImu::sample() {
    uint8_t chunk[128];
    for (;;) {
        const int n = uart_read_bytes(port_, chunk, sizeof(chunk), 0);
        if (n <= 0) break;
        feed(chunk, static_cast<size_t>(n));
        if (n < static_cast<int>(sizeof(chunk))) break;
    }
}

void RvcImu::feed(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        const uint8_t b = data[i];

        // Resynchronise on the header pair rather than trusting alignment, a
        // single dropped byte would otherwise corrupt every later frame.
        if (len_ < 2) {
            if (b == kRvcHeader) buf_[len_++] = b;
            else len_ = 0;
            continue;
        }

        buf_[len_++] = b;
        if (len_ < kRvcFrameLen) continue;

        uint8_t sum = 0;
        for (uint8_t k = 2; k < 18; ++k) sum = static_cast<uint8_t>(sum + buf_[k]);

        if (sum == buf_[18]) onFrame(buf_);
        else ++badChecksums_;

        len_ = 0;
    }
}

void RvcImu::onFrame(const uint8_t* f) {
    const int16_t raw = static_cast<int16_t>(static_cast<uint16_t>(f[3]) |
                                             (static_cast<uint16_t>(f[4]) << 8));
    const double yawDeg = raw * kRvcYawLsbDeg;   // wrapped, CCW+

    if (!seeded_) {
        rawYawDeg_ = yawDeg;
        seeded_ = true;
    }

    const double delta = yawDeg - rawYawDeg_;
    if (delta > 180.0) revolutions_ -= 1.0;
    else if (delta < -180.0) revolutions_ += 1.0;

    rawYawDeg_ = yawDeg;

    // Negated because RVC is counter-clockwise positive and everything in
    // gflib is clockwise positive.
    headingDeg_ = -(rawYawDeg_ + 360.0 * revolutions_);

    ++frames_;
    lastFrameMs_ = static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

bool RvcImu::ok(uint32_t nowMs) const {
    if (frames_ == 0) return false;
    return (nowMs - lastFrameMs_) <= cfg::kImuFrameTimeoutMs;
}

// rs-485

namespace {
constexpr int kLinkRxBufBytes = 1024;
constexpr int kLinkTxBufBytes = 512;
}  // namespace

Rs485Stream::Rs485Stream(uart_port_t port, int txPin, int rxPin, int dePin)
    : port_(port), txPin_(txPin), rxPin_(rxPin), dePin_(dePin) {}

esp_err_t Rs485Stream::begin() {
    uart_config_t c = {};
    c.baud_rate = static_cast<int>(gflib::kLinkBaud);
    c.data_bits = UART_DATA_8_BITS;
    c.parity = UART_PARITY_DISABLE;
    c.stop_bits = UART_STOP_BITS_1;
    c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    c.source_clk = UART_SCLK_DEFAULT;

    ESP_RETURN_ON_ERROR(uart_driver_install(port_, kLinkRxBufBytes, kLinkTxBufBytes, 0, nullptr, 0),
                        kTag, "link driver");
    ESP_RETURN_ON_ERROR(uart_param_config(port_, &c), kTag, "link config");
    ESP_RETURN_ON_ERROR(uart_set_pin(port_, txPin_, rxPin_, dePin_, UART_PIN_NO_CHANGE),
                        kTag, "link pins");

    // the peripheral raises and drops DE around the transmission, timed off
    // the last stop bit rather than off a software guess.
    // Note: this is why esp idf is used and not Arduino.
    ESP_RETURN_ON_ERROR(uart_set_mode(port_, UART_MODE_RS485_HALF_DUPLEX), kTag, "rs485 mode");

    return ESP_OK;
}

size_t Rs485Stream::read(uint8_t* dst, size_t cap) {
    const int n = uart_read_bytes(port_, dst, static_cast<uint32_t>(cap), 0);
    return n > 0 ? static_cast<size_t>(n) : 0;
}

size_t Rs485Stream::readBlocking(uint8_t* dst, size_t cap, int64_t timeoutUs) {
    if (cap == 0) return 0;

    TickType_t ticks = pdMS_TO_TICKS(timeoutUs / 1000);
    if (ticks == 0) ticks = 1;

    // One byte with a timeout, then the rest with none. Waiting on the first
    // byte alone means the return lands on its actual arrival rather than at
    // the end of a fixed window
    const int first = uart_read_bytes(port_, dst, 1, ticks);
    if (first != 1) return 0;

    size_t n = 1;
    if (cap > 1) {
        const int rest = uart_read_bytes(port_, dst + 1, static_cast<uint32_t>(cap - 1), 0);
        if (rest > 0) n += static_cast<size_t>(rest);
    }
    return n;
}

size_t Rs485Stream::write(const uint8_t* src, size_t len) {
    size_t free = 0;
    if (uart_get_tx_buffer_free_size(port_, &free) != ESP_OK) return 0;
    if (free < len) return 0;

    const int n = uart_write_bytes(port_, src, len);
    return n > 0 ? static_cast<size_t>(n) : 0;
}

}  // namespace hal
