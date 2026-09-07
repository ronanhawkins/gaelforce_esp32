#include "tmf8821.hpp"

#include <cstring>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tof_firmware.hpp"

namespace tof {

namespace {

const char* kTag = "tof";

// registers -- DS000693 section 8
constexpr uint8_t kRegAppId        = 0x00;
constexpr uint8_t kRegMinor        = 0x01;
constexpr uint8_t kRegCalibStatus  = 0x07;
constexpr uint8_t kRegCmdStat      = 0x08;
constexpr uint8_t kRegConfigResult = 0x20;   // cid_rid, then tid, size
constexpr uint8_t kRegPeriodMs     = 0x24;
constexpr uint8_t kRegKiloIter     = 0x26;
constexpr uint8_t kRegConfThresh   = 0x30;
constexpr uint8_t kRegSpadMapId    = 0x34;
constexpr uint8_t kRegI2cAddr      = 0x3B;
constexpr uint8_t kRegI2cAddrChg   = 0x3E;
constexpr uint8_t kRegEnable       = 0xE0;
constexpr uint8_t kRegIntStatus    = 0xE1;
constexpr uint8_t kRegIntEnab      = 0xE2;

// application ids
constexpr uint8_t kAppMeasure    = 0x03;
constexpr uint8_t kAppBootloader = 0x80;

// commands written to CMD_STAT
constexpr uint8_t kCmdMeasure         = 0x10;
constexpr uint8_t kCmdWriteConfigPage = 0x15;
constexpr uint8_t kCmdLoadCommonPage  = 0x16;
constexpr uint8_t kCmdI2cAddress      = 0x21;
constexpr uint8_t kCmdStop            = 0xFF;

// Pending while CMD_STAT reads >= 0x10; below that it is a status.
constexpr uint8_t kStatusOk       = 0x00;
constexpr uint8_t kStatusAccepted = 0x01;
constexpr uint8_t kStatusPending  = 0x10;

// bootloader commands -- AN001015 section 3
constexpr uint8_t kBlRamRemapReset = 0x11;
constexpr uint8_t kBlDownloadInit  = 0x14;
constexpr uint8_t kBlWriteRam      = 0x41;
constexpr uint8_t kBlSetAddr       = 0x43;
constexpr uint8_t kBlChunkBytes    = 128;

// cid_rid values for the paged window at 0x24..0xDF
constexpr uint8_t kPageMeasureResult = 0x10;
constexpr uint8_t kPageCommonConfig  = 0x16;

// INT_STATUS bit 1 -- measurement result ready. Write-1-to-clear.
constexpr uint8_t kIntResult = 0x02;

// ENABLE bits: cpu_ready (6) and pon (0).
constexpr uint8_t kEnableCpuReady = 0x40;
constexpr uint8_t kEnablePon      = 0x01;

// cid_rid at 0x20 through object 1 of zone 18 at 0x6D. Object 2 is not read.
constexpr uint8_t kResultBase  = kRegConfigResult;
constexpr size_t  kResultBytes = 78;
constexpr size_t  kZoneBase    = 0x38 - kResultBase;

constexpr int kXferTimeoutMs = 100;

// EN must stay low ≥1 ms to force a cold start (AN001015 §2).
constexpr uint32_t kResetHoldMs = 5;

// ACK-poll deadline after power-on.
constexpr uint32_t kAckTimeoutMs = 50;

// Every un-enumerated part answers here -- hence one at a time.
constexpr uint16_t kBootAddress = 0x41;

// One's complement of CMD_STAT + SIZE + data.
uint8_t blChecksum(const uint8_t* p, size_t n) {
    uint32_t sum = 0;
    for (size_t i = 0; i < n; ++i) sum += p[i];
    return static_cast<uint8_t>(~sum);
}

uint32_t le32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

bool expired(int64_t startUs, uint32_t timeoutMs) {
    return (esp_timer_get_time() - startUs) > static_cast<int64_t>(timeoutMs) * 1000;
}

}  // namespace

Tmf8821::Tmf8821(int enPin, uint8_t runAddr) : enPin_(enPin), runAddr_(runAddr) {}

// i2c primitives

esp_err_t Tmf8821::wr(uint8_t reg, const uint8_t* data, size_t n) {
    // One transaction: a STOP between register and payload loses the pointer.
    uint8_t buf[kBlChunkBytes + 8];
    if (n + 1 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    buf[0] = reg;
    if (n) std::memcpy(buf + 1, data, n);

    const esp_err_t e = i2c_master_transmit(dev_, buf, n + 1, kXferTimeoutMs);
    if (e != ESP_OK) ++busErrors_;
    return e;
}

esp_err_t Tmf8821::wr8(uint8_t reg, uint8_t v) { return wr(reg, &v, 1); }

esp_err_t Tmf8821::rd(uint8_t reg, uint8_t* dst, size_t n) {
    const esp_err_t e =
        i2c_master_transmit_receive(dev_, &reg, 1, dst, n, kXferTimeoutMs);
    if (e != ESP_OK) ++busErrors_;
    return e;
}

esp_err_t Tmf8821::rd8(uint8_t reg, uint8_t& v) { return rd(reg, &v, 1); }

// Deadline polling

esp_err_t Tmf8821::waitCpuReady(uint32_t timeoutMs) {
    const int64_t start = esp_timer_get_time();
    for (;;) {
        uint8_t v = 0;
        if (rd8(kRegEnable, v) == ESP_OK &&
            (v & kEnableCpuReady) && (v & kEnablePon)) {
            return ESP_OK;
        }
        if (expired(start, timeoutMs)) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t Tmf8821::waitCommand(uint8_t& status, uint32_t timeoutMs) {
    const int64_t start = esp_timer_get_time();
    for (;;) {
        uint8_t v = 0;
        if (rd8(kRegCmdStat, v) == ESP_OK && v < kStatusPending) {
            status = v;
            return ESP_OK;
        }
        if (expired(start, timeoutMs)) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t Tmf8821::waitBootloaderReady(uint32_t timeoutMs) {
    const int64_t start = esp_timer_get_time();
    for (;;) {
        // Completed command answers 00 00 FF: status 0, size 0, checksum.
        uint8_t v[3] = {};
        if (rd(kRegCmdStat, v, sizeof(v)) == ESP_OK &&
            v[0] == 0x00 && v[1] == 0x00 && v[2] == 0xFF) {
            return ESP_OK;
        }
        if (expired(start, timeoutMs)) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

esp_err_t Tmf8821::waitAppId(uint8_t want, uint32_t timeoutMs) {
    const int64_t start = esp_timer_get_time();
    for (;;) {
        uint8_t v = 0;
        if (rd8(kRegAppId, v) == ESP_OK && v == want) return ESP_OK;
        if (expired(start, timeoutMs)) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// bring-up

esp_err_t Tmf8821::holdReset() {
    gpio_config_t c = {};
    c.pin_bit_mask = 1ULL << enPin_;
    c.mode = GPIO_MODE_OUTPUT;
    c.pull_up_en = GPIO_PULLUP_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_DISABLE;
    c.intr_type = GPIO_INTR_DISABLE;
    ESP_RETURN_ON_ERROR(gpio_config(&c), kTag, "en pin %d", enPin_);

    present_ = false;
    return gpio_set_level(static_cast<gpio_num_t>(enPin_), 0);
}

esp_err_t Tmf8821::bringUp(i2c_master_bus_handle_t bus, uint32_t sclHz) {
    bus_ = bus;
    present_ = false;

    // Reassert the cold-start pulse width.
    ESP_RETURN_ON_ERROR(gpio_set_level(static_cast<gpio_num_t>(enPin_), 0), kTag, "en low");
    vTaskDelay(pdMS_TO_TICKS(kResetHoldMs));

    // Keep other devices disabled until this one leaves the boot address.
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = kBootAddress;
    dc.scl_speed_hz = sclHz;
    if (dev_ != nullptr) {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_, &dc, &dev_), kTag, "add dev");
    sclHz_ = sclHz;

    ESP_RETURN_ON_ERROR(gpio_set_level(static_cast<gpio_num_t>(enPin_), 1), kTag, "en high");

    // Probe until registers become readable.
    {
        const int64_t start = esp_timer_get_time();
        for (;;) {
            if (i2c_master_probe(bus_, kBootAddress, 10) == ESP_OK) break;
            if (expired(start, kAckTimeoutMs)) {
                ESP_LOGE(kTag, "en=%d: no ACK at 0x%02X within %ums -- check EN wiring",
                         enPin_, kBootAddress, static_cast<unsigned>(kAckTimeoutMs));
                return ESP_ERR_TIMEOUT;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    // Cold-start bits 5:4 select the bootloader (AN001015 §1.2.1).
    ESP_RETURN_ON_ERROR(wr8(kRegEnable, kEnablePon), kTag, "enable");
    ESP_RETURN_ON_ERROR(waitCpuReady(50), kTag, "cpu never became ready");

    uint8_t appId = 0;
    ESP_RETURN_ON_ERROR(rd8(kRegAppId, appId), kTag, "appid");
    if (appId != kAppBootloader) {
        // A different app ID means the cold reset failed.
        ESP_LOGE(kTag, "en=%d: expected bootloader 0x80 after cold start, got 0x%02X",
                 enPin_, appId);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(downloadFirmware(), kTag, "firmware download");
    ESP_RETURN_ON_ERROR(rd8(kRegMinor, minor_), kTag, "minor");
    ESP_RETURN_ON_ERROR(rd8(kRegCalibStatus, calStatus_), kTag, "calib status");
    ESP_RETURN_ON_ERROR(assignAddress(), kTag, "address change");

    present_ = true;
    return ESP_OK;
}

esp_err_t Tmf8821::downloadFirmware() {
    uint8_t cmd[3 + kBlChunkBytes + 1];

    cmd[0] = kBlDownloadInit;
    cmd[1] = 0x01;
    cmd[2] = 0x29;                       // ROM v2 seed, per AN001015 3.2
    cmd[3] = blChecksum(cmd, 3);
    ESP_RETURN_ON_ERROR(wr(kRegCmdStat, cmd, 4), kTag, "download_init");
    ESP_RETURN_ON_ERROR(waitBootloaderReady(100), kTag, "download_init ready");

    cmd[0] = kBlSetAddr;
    cmd[1] = 0x02;
    cmd[2] = static_cast<uint8_t>(kFirmwareLoadAddr & 0xFF);
    cmd[3] = static_cast<uint8_t>(kFirmwareLoadAddr >> 8);
    cmd[4] = blChecksum(cmd, 4);
    ESP_RETURN_ON_ERROR(wr(kRegCmdStat, cmd, 5), kTag, "set_addr");
    ESP_RETURN_ON_ERROR(waitBootloaderReady(100), kTag, "set_addr ready");

    // One contiguous block: no SET_ADDR between chunks.
    for (size_t off = 0; off < kFirmwareImageLen; off += kBlChunkBytes) {
        const size_t n = (kFirmwareImageLen - off) < kBlChunkBytes
                             ? (kFirmwareImageLen - off)
                             : kBlChunkBytes;
        cmd[0] = kBlWriteRam;
        cmd[1] = static_cast<uint8_t>(n);
        std::memcpy(cmd + 2, kFirmwareImage + off, n);
        cmd[2 + n] = blChecksum(cmd, 2 + n);

        ESP_RETURN_ON_ERROR(wr(kRegCmdStat, cmd, 3 + n), kTag, "w_ram @%u",
                            static_cast<unsigned>(off));

        // Wait: early commands are silently discarded.
        ESP_RETURN_ON_ERROR(waitBootloaderReady(100), kTag, "w_ram ready @%u",
                            static_cast<unsigned>(off));
    }

    cmd[0] = kBlRamRemapReset;
    cmd[1] = 0x00;
    cmd[2] = blChecksum(cmd, 2);
    ESP_RETURN_ON_ERROR(wr(kRegCmdStat, cmd, 3), kTag, "ramremap_reset");

    // No handshake after this one -- the part resets into the image.
    ESP_RETURN_ON_ERROR(waitAppId(kAppMeasure, 500), kTag,
                        "application never started -- image incomplete");
    return waitCpuReady(50);
}

esp_err_t Tmf8821::loadCommonPage() {
    ESP_RETURN_ON_ERROR(wr8(kRegCmdStat, kCmdLoadCommonPage), kTag, "load page");

    uint8_t status = 0xFF;
    ESP_RETURN_ON_ERROR(waitCommand(status, 100), kTag, "load page wait");
    if (status != kStatusOk) return ESP_ERR_INVALID_RESPONSE;

    // Verify the shared window contains the config page.
    uint8_t cid = 0;
    ESP_RETURN_ON_ERROR(rd8(kRegConfigResult, cid), kTag, "cid");
    return cid == kPageCommonConfig ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t Tmf8821::writeCommonPage() {
    ESP_RETURN_ON_ERROR(wr8(kRegCmdStat, kCmdWriteConfigPage), kTag, "write page");

    uint8_t status = 0xFF;
    ESP_RETURN_ON_ERROR(waitCommand(status, 100), kTag, "write page wait");
    return status == kStatusOk ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

esp_err_t Tmf8821::assignAddress() {
    ESP_RETURN_ON_ERROR(loadCommonPage(), kTag, "load page for address");

    // 7-bit address shifted UP one place; bit 0 is the r/w slot.
    ESP_RETURN_ON_ERROR(wr8(kRegI2cAddr, static_cast<uint8_t>(runAddr_ << 1)),
                        kTag, "addr reg");

    // Zero applies the address change unconditionally.
    ESP_RETURN_ON_ERROR(wr8(kRegI2cAddrChg, 0x00), kTag, "addr change gate");
    ESP_RETURN_ON_ERROR(writeCommonPage(), kTag, "write page for address");

    // The command status appears at the new address.
    ESP_RETURN_ON_ERROR(wr8(kRegCmdStat, kCmdI2cAddress), kTag, "addr command");
    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_RETURN_ON_ERROR(
        i2c_master_device_change_address(dev_, runAddr_, kXferTimeoutMs), kTag,
        "repoint handle");

    // Verify the device moved.
    ESP_RETURN_ON_ERROR(waitAppId(kAppMeasure, 100), kTag,
                        "no answer at new address 0x%02X", runAddr_);
    return ESP_OK;
}

esp_err_t Tmf8821::setSclHz(uint32_t hz) {
    if (dev_ == nullptr || bus_ == nullptr) return ESP_ERR_INVALID_STATE;
    if (hz == sclHz_) return ESP_OK;

    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = runAddr_;
    dc.scl_speed_hz = hz;

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(dev_), kTag, "rm dev");
    dev_ = nullptr;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_, &dc, &dev_), kTag, "re-add dev");
    sclHz_ = hz;
    return ESP_OK;
}

// measurement

esp_err_t Tmf8821::configure(uint16_t periodMs, uint16_t kiloIterations,
                             uint8_t spadMapId, uint8_t confidenceThreshold) {
    if (!present_) return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(loadCommonPage(), kTag, "load page for config");

    const uint8_t period[2] = {static_cast<uint8_t>(periodMs & 0xFF),
                               static_cast<uint8_t>(periodMs >> 8)};
    ESP_RETURN_ON_ERROR(wr(kRegPeriodMs, period, 2), kTag, "period");

    const uint8_t iter[2] = {static_cast<uint8_t>(kiloIterations & 0xFF),
                             static_cast<uint8_t>(kiloIterations >> 8)};
    ESP_RETURN_ON_ERROR(wr(kRegKiloIter, iter, 2), kTag, "iterations");

    ESP_RETURN_ON_ERROR(wr8(kRegConfThresh, confidenceThreshold), kTag, "conf thresh");
    ESP_RETURN_ON_ERROR(wr8(kRegSpadMapId, spadMapId), kTag, "spad map");
    ESP_RETURN_ON_ERROR(writeCommonPage(), kTag, "write config");

    // Disable the unwired interrupt pin; status still updates.
    ESP_RETURN_ON_ERROR(wr8(kRegIntEnab, 0x00), kTag, "int enab");
    ESP_RETURN_ON_ERROR(wr8(kRegIntStatus, 0xFF), kTag, "clear stale ints");

    // A SPAD map without matching calibration is reported here, not refused.
    ESP_RETURN_ON_ERROR(rd8(kRegCalibStatus, calStatus_), kTag, "calib status");
    return ESP_OK;
}

esp_err_t Tmf8821::startMeasuring() {
    if (!present_) return ESP_ERR_INVALID_STATE;

    ESP_RETURN_ON_ERROR(wr8(kRegCmdStat, kCmdMeasure), kTag, "measure");

    uint8_t status = 0xFF;
    ESP_RETURN_ON_ERROR(waitCommand(status, 100), kTag, "measure wait");

    // Long-running MEASURE succeeds with ACCEPTED.
    if (status != kStatusAccepted) {
        ESP_LOGE(kTag, "0x%02X: MEASURE rejected, CMD_STAT 0x%02X", runAddr_, status);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t Tmf8821::stopMeasuring() {
    if (!present_) return ESP_ERR_INVALID_STATE;
    ESP_RETURN_ON_ERROR(wr8(kRegCmdStat, kCmdStop), kTag, "stop");

    uint8_t status = 0xFF;
    return waitCommand(status, 100);
}

bool Tmf8821::resultReady() {
    if (!present_) return false;
    uint8_t v = 0;
    if (rd8(kRegIntStatus, v) != ESP_OK) return false;
    return (v & kIntResult) != 0;
}

bool Tmf8821::readResult(RawResult& out) {
    if (!present_) return false;

    // Clear before reading to preserve a concurrently published interrupt.
    if (wr8(kRegIntStatus, kIntResult) != ESP_OK) return false;

    uint8_t buf[kResultBytes];
    if (rd(kResultBase, buf, sizeof(buf)) != ESP_OK) return false;

    // A config page sitting in the window decodes as plausible distances.
    if (buf[0] != kPageMeasureResult) return false;

    out.tid          = buf[1];
    out.resultNumber = buf[4];
    out.temperatureC = buf[5];
    out.validResults = buf[6];
    out.ambient      = le32(&buf[8]);
    out.sysTick      = le32(&buf[20]);

    for (int z = 0; z < kZoneCount; ++z) {
        const size_t b = kZoneBase + 3 * static_cast<size_t>(z);
        out.zones[z].confidence = buf[b];
        out.zones[z].distanceMm =
            static_cast<uint16_t>(buf[b + 1] | (static_cast<uint16_t>(buf[b + 2]) << 8));
    }
    return true;
}

}  // namespace tof
