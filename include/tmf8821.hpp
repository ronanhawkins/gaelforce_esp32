#pragma once
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

// TMF8821 transport for a shared Qwiic bus.
// Register references: DS000693 v4-00 and AN001015 v3-00.

namespace tof {

// 18 slots, but 4x4 fills only 1-8 and 10-17 with ONE object each. Two objects
// per zone exist only in 3x3, where 9 zones x 2 fill all 18.
inline constexpr int kZoneCount = 18;

// How many are fetched. The rest stay confidence 0, which already means
// no-target, so reading past this sees an absent zone rather than a wrong one.
inline constexpr int kZonesRead = 8;

struct ZoneResult {
    // Confidence 0 means no target.
    uint8_t  confidence = 0;
    uint16_t distanceMm = 0;
};

struct RawResult {
    // Increments for each published record.
    uint8_t  tid = 0;

    uint8_t  resultNumber = 0;
    uint8_t  temperatureC = 0;

    // Zones reporting records, including no-target records.
    uint8_t  validResults = 0;

    uint32_t ambient = 0;
    uint32_t sysTick = 0;

    // Index 0 is zone 1.
    ZoneResult zones[kZoneCount] = {};
};

class Tmf8821 {
    public:
        Tmf8821(int enPin, uint8_t runAddr);

        // Cold-reset every device before bringing any device up.
        esp_err_t holdReset();

        // Boot, load firmware, and assign runAddr one device at a time.
        esp_err_t bringUp(i2c_master_bus_handle_t bus, uint32_t sclHz);

        // Rebuild the device handle at a new bus speed.
        esp_err_t setSclHz(uint32_t hz);

        // Replay pre-encoded SPAD page bytes. Call before configure() when
        // spad_map_id selects a user-defined map, or it runs with no mask.
        esp_err_t downloadSpadMask(const uint8_t* blob, size_t n);

        // Configure through one config-page cycle.
        esp_err_t configure(uint16_t periodMs, uint16_t kiloIterations,
                            uint8_t spadMapId, uint8_t confidenceThreshold);

        esp_err_t startMeasuring();
        esp_err_t stopMeasuring();

        // Poll INT_STATUS without dedicated interrupt GPIOs.
        bool resultReady();

        // Clear then read atomically to avoid straddling records (AN001015 4.4).
        bool readResult(RawResult& out);

        // True only after complete bring-up.
        bool present() const { return present_; }

        // Returns 0 when absent.
        uint8_t address() const { return present_ ? runAddr_ : uint8_t{0}; }

        // Register 0x07; 0x31 indicates no factory calibration.
        uint8_t calibrationStatus() const { return calStatus_; }

        uint8_t appMinorVersion() const { return minor_; }
        uint32_t busErrors() const { return busErrors_; }

    private:
        esp_err_t wr(uint8_t reg, const uint8_t* data, size_t n);
        esp_err_t wr8(uint8_t reg, uint8_t v);
        esp_err_t rd(uint8_t reg, uint8_t* dst, size_t n);
        esp_err_t rd8(uint8_t reg, uint8_t& v);

        esp_err_t waitCpuReady(uint32_t timeoutMs);
        esp_err_t waitCommand(uint8_t& status, uint32_t timeoutMs);
        esp_err_t waitBootloaderReady(uint32_t timeoutMs);
        esp_err_t waitAppId(uint8_t want, uint32_t timeoutMs);

        esp_err_t downloadFirmware();
        esp_err_t assignAddress();
        esp_err_t loadCommonPage();
        esp_err_t loadSpadPage();

        // CMD_WRITE_CONFIG_PAGE writes whichever page is currently loaded.
        esp_err_t writeLoadedPage();

        int      enPin_;
        uint8_t  runAddr_;

        i2c_master_bus_handle_t bus_ = nullptr;
        i2c_master_dev_handle_t dev_ = nullptr;
        uint32_t sclHz_ = 0;

        bool     present_ = false;
        uint8_t  calStatus_ = 0xFF;
        uint8_t  minor_ = 0;
        uint32_t busErrors_ = 0;
};

}  // namespace tof
