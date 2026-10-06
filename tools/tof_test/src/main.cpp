// TMF8821 bench test for the pod's four ToF sensors.
//
// Walks the same bring-up the pod does, one stage at a time, and says which
// stage failed for which sensor. Then streams live distances so each sensor can
// be checked with a hand or a wall in front of it.
//
// Pins, addresses and the SPAD map are copied from include/pod_config.hpp,
// which cannot be included here without gflib. Keep them in step.

#include <cinttypes>
#include <cmath>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tmf8821.hpp"

namespace {

// pod_config.hpp copies

constexpr int kSdaPin = 16;
constexpr int kSclPin = 15;

constexpr int kCount = 4;
constexpr const char* kName[kCount] = {"FRONT", "RIGHT", "REAR ", "LEFT "};
constexpr int kEnPins[kCount] = {4, 12, 1, 2};
constexpr uint8_t kAddrs[kCount] = {0x42, 0x43, 0x44, 0x45};

constexpr uint32_t kEnumHz = 100000;
constexpr uint32_t kRunHz = 400000;

constexpr uint16_t kPeriodMs = 70;
constexpr uint16_t kKiloIterations = 1000;
constexpr uint8_t kConfThreshold = 6;
constexpr uint32_t kStaggerMs = 17;

// Set false to fall back to the stock 3x3 map (id 1) if the custom mask is the
// suspect. Zones 2/4/6/8 then mean something different, but any non-zero
// distance still proves the sensor ranges.
constexpr bool kUseCustomMap = true;
constexpr uint8_t kCustomSpadMap = 14;
constexpr uint8_t kStockSpadMap = 1;

constexpr uint8_t kSpadMaskBlob[] = {
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

constexpr int kZones[4] = {2, 4, 6, 8};

// test state

tof::Tmf8821 g_dev[kCount] = {
    {kEnPins[0], kAddrs[0]}, {kEnPins[1], kAddrs[1]},
    {kEnPins[2], kAddrs[2]}, {kEnPins[3], kAddrs[3]},
};

struct Live {
    bool up = false;
    const char* failedAt = "not tried";
    esp_err_t err = ESP_OK;
    tof::RawResult last{};
    bool haveResult = false;
    uint32_t results = 0;
    uint32_t readFails = 0;
    int64_t lastResultUs = 0;
};
Live g_live[kCount];

void banner(const char* s) { printf("\n==== %s ====\n", s); }

// Before the driver claims the pins: a line that idles low has no pull-up, or
// is shorted to ground, and nothing after this will work.
bool checkIdleLines() {
    banner("1. I2C idle levels (both should be 1)");
    gpio_config_t c = {};
    c.pin_bit_mask = (1ULL << kSdaPin) | (1ULL << kSclPin);
    c.mode = GPIO_MODE_INPUT;
    c.pull_up_en = GPIO_PULLUP_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&c);
    vTaskDelay(pdMS_TO_TICKS(2));

    const int sda = gpio_get_level(static_cast<gpio_num_t>(kSdaPin));
    const int scl = gpio_get_level(static_cast<gpio_num_t>(kSclPin));
    printf("SDA (GPIO%d) = %d   SCL (GPIO%d) = %d\n", kSdaPin, sda, kSclPin, scl);
    if (!sda || !scl) {
        printf("FAIL: a line idles LOW. No pull-up reaching it (breakout pull-up\n"
               "      jumpers cut, or sensor unpowered), or the line is shorted.\n");
        return false;
    }
    printf("OK\n");
    return true;
}

void scan(i2c_master_bus_handle_t bus, const char* label) {
    printf("scan (%s):", label);
    int found = 0;
    for (uint16_t a = 0x08; a < 0x78; ++a) {
        if (i2c_master_probe(bus, a, 20) == ESP_OK) {
            printf(" 0x%02X", a);
            ++found;
        }
    }
    printf(found ? "\n" : " nothing\n");
}

void bringUpAll(i2c_master_bus_handle_t bus) {
    banner("3. Bring-up, one sensor at a time");
    for (int i = 0; i < kCount; ++i) {
        printf("\n-- tof%d %s  EN=GPIO%d  -> 0x%02X\n", i, kName[i], kEnPins[i], kAddrs[i]);

        const esp_err_t e = g_dev[i].bringUp(bus, kEnumHz);
        if (e != ESP_OK) {
            g_dev[i].holdReset();
            g_live[i].failedAt = "boot/firmware/address (see error above)";
            printf("FAIL: %s\n", esp_err_to_name(e));
            continue;
        }
        printf("booted, firmware loaded, address set. app minor 0x%02X, calib 0x%02X%s\n",
               g_dev[i].appMinorVersion(), g_dev[i].calibrationStatus(),
               g_dev[i].calibrationStatus() == 0 ? "" : " (no factory cal, expected)");
    }

    scan(bus, "after bring-up, expect 0x42..0x45 for working sensors");

    banner("4. Configure and start measuring");
    for (int i = 0; i < kCount; ++i) {
        if (!g_dev[i].present()) continue;

        esp_err_t e = g_dev[i].setSclHz(kRunHz);
        if (e != ESP_OK) { g_live[i].failedAt = "400kHz switch"; goto fail; }

        // Same sequence as TofArray::begin(): select the map, then the mask,
        // then configure again. Mask-first fails on these parts.
        e = g_dev[i].configure(kPeriodMs, kKiloIterations,
                               kUseCustomMap ? kCustomSpadMap : kStockSpadMap,
                               kConfThreshold);
        if (e != ESP_OK) { g_live[i].failedAt = "configure"; goto fail; }

        if (kUseCustomMap) {
            e = g_dev[i].downloadSpadMask(kSpadMaskBlob, sizeof(kSpadMaskBlob));
            if (e != ESP_OK) { g_live[i].failedAt = "SPAD mask download"; goto fail; }
            e = g_dev[i].configure(kPeriodMs, kKiloIterations, kCustomSpadMap,
                                   kConfThreshold);
            if (e != ESP_OK) { g_live[i].failedAt = "configure after mask"; goto fail; }
        }

        e = g_dev[i].startMeasuring();
        if (e != ESP_OK) { g_live[i].failedAt = "start measuring"; goto fail; }

        g_live[i].up = true;
        g_live[i].failedAt = "";
        printf("tof%d %s measuring\n", i, kName[i]);
        vTaskDelay(pdMS_TO_TICKS(kStaggerMs));
        continue;

    fail:
        g_live[i].err = e;
        printf("tof%d %s FAIL at %s: %s\n", i, kName[i], g_live[i].failedAt,
               esp_err_to_name(e));
        g_dev[i].holdReset();
    }
}

// One distance per sensor: the median of the zones that saw a target, the same
// reduction the pod uses (minus its projection and gates).
int distanceMm(const Live& L) {
    if (!L.haveResult) return -1;
    int v[4];
    int n = 0;
    for (int z = 0; z < 4; ++z) {
        const tof::ZoneResult& zr = L.last.zones[kZones[z] - 1];
        if (zr.confidence == 0 || zr.distanceMm == 0) continue;
        int k = n++;
        while (k > 0 && v[k - 1] > zr.distanceMm) { v[k] = v[k - 1]; --k; }
        v[k] = zr.distanceMm;
    }
    if (n == 0) return -1;
    return (n & 1) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

void printDistances() {
    const int64_t nowUs = esp_timer_get_time();
    for (int i = 0; i < kCount; ++i) {
        const Live& L = g_live[i];
        printf("%s ", kName[i]);
        if (!L.up) printf("   DEAD   ");
        else if (L.haveResult && nowUs - L.lastResultUs > 500000) printf("  STALLED ");
        else {
            const int mm = distanceMm(L);
            if (mm < 0) printf("    --    ");
            else printf("%5d mm  ", mm);
        }
        if (i < kCount - 1) printf("|  ");
    }
    printf("\n");
}

}  // namespace

extern "C" void app_main(void) {
    // Every sensor off before anything touches the bus: they all boot at 0x41.
    for (int i = 0; i < kCount; ++i) g_dev[i].holdReset();

    vTaskDelay(pdMS_TO_TICKS(500));   // let the monitor attach
    printf("\n\nGaelforce TMF8821 bench test\n");
    printf("SDA GPIO%d, SCL GPIO%d, EN %d/%d/%d/%d, map %s\n", kSdaPin, kSclPin,
           kEnPins[0], kEnPins[1], kEnPins[2], kEnPins[3],
           kUseCustomMap ? "custom 14" : "stock 1 (3x3)");

    const bool linesOk = checkIdleLines();

    banner("2. I2C bus");
    i2c_master_bus_config_t bc = {};
    bc.i2c_port = I2C_NUM_0;
    bc.sda_io_num = static_cast<gpio_num_t>(kSdaPin);
    bc.scl_io_num = static_cast<gpio_num_t>(kSclPin);
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    // Off, as on the pod: the test must fail the same way if the pull-ups are wrong.
    bc.flags.enable_internal_pullup = false;

    i2c_master_bus_handle_t bus = nullptr;
    const esp_err_t be = i2c_new_master_bus(&bc, &bus);
    if (be != ESP_OK) {
        printf("FAIL: i2c_new_master_bus: %s\n", esp_err_to_name(be));
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    if (!linesOk) printf("(continuing anyway)\n");

    // Re-run while anything is dead, so the setup log is never only at boot,
    // where a USB monitor that attaches late misses it.
    constexpr int64_t kRetryUs = 10 * 1000000LL;
    int64_t lastSetupUs = 0;
    int runs = 0;

    auto setup = [&] {
        ++runs;
        printf("\n\n######## setup run %d ########\n", runs);
        for (int i = 0; i < kCount; ++i) {
            g_dev[i].holdReset();
            g_live[i] = Live{};
        }
        vTaskDelay(pdMS_TO_TICKS(10));

        // All held in reset, so nothing should answer. Something at 0x41 means
        // a sensor whose EN is not wired, or is wired to the wrong GPIO.
        scan(bus, "all EN low, expect nothing");

        bringUpAll(bus);

        int up = 0;
        for (int i = 0; i < kCount; ++i) if (g_live[i].up) ++up;
        banner("5. Live distances (wave a hand ~10-30cm in front of each)");
        printf("%d of %d sensors measuring%s\n", up, kCount,
               up < kCount ? " -- full setup re-runs every 10s" : "");
        lastSetupUs = esp_timer_get_time();
        return up;
    };

    int up = setup();

    int64_t lastPrintUs = 0;
    for (;;) {
        if (up < kCount && esp_timer_get_time() - lastSetupUs > kRetryUs) up = setup();

        for (int i = 0; i < kCount; ++i) {
            if (!g_live[i].up || !g_dev[i].resultReady()) continue;
            tof::RawResult r;
            if (!g_dev[i].readResult(r)) { ++g_live[i].readFails; continue; }
            if (g_live[i].haveResult && r.tid == g_live[i].last.tid) continue;
            g_live[i].last = r;
            g_live[i].haveResult = true;
            ++g_live[i].results;
            g_live[i].lastResultUs = esp_timer_get_time();
        }

        if (esp_timer_get_time() - lastPrintUs > 250000) {
            lastPrintUs = esp_timer_get_time();
            printDistances();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
