#pragma once
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "gflib/mcl.hpp"
#include "gflib/pose.hpp"
#include "pod_config.hpp"
#include "tmf8821.hpp"

// Enumerates and schedules four sensors, reducing raw zones to SensorReadings.

namespace tof {

using gflib::real;
using gflib::operator""_r;

// Per-sensor rejection reasons for diagnostics.
enum class Reject : uint8_t {
    None = 0,
    Absent,       // never enumerated
    NoReturn,     // too few zones reported a target
    OutOfRange,   // past the range where sensorSigmaInches is still honest
    Dispersion,   // zones disagree; no single flat surface fits
    Oblique,      // grazing incidence against the wall it should be facing
    Motion,       // robot moving hard enough to smear the integration
};

struct SensorStatus {
    bool     present = false;
    uint8_t  address = 0;
    uint8_t  calibrationStatus = 0xFF;
    uint8_t  appMinor = 0;
    uint32_t busErrors = 0;

    // Last completed measurement, whatever became of it.
    uint16_t lastZoneMm[4] = {};      // raw, per configured zone, pre-projection
    uint8_t  lastZoneConf[4] = {};
    real     lastMedianInches = 0.0_r;
    bool     lastValid = false;
    Reject   lastReject = Reject::Absent;

    // Milliseconds from boot to first result.
    uint32_t firstResultMs = 0;

    uint32_t results = 0;             // records actually read
    uint32_t accepted = 0;
    uint32_t rejects[7] = {};         // indexed by Reject

    // Incidence checks skipped because the mount estimate was outside the field.
    uint32_t incidenceUncomputable = 0;
};

static_assert(static_cast<int>(Reject::Motion) == 6,
              "SensorStatus::rejects must have one slot per Reject");

// This layer owns neither pose nor link state.
struct GateContext {
    gflib::Pose estimate{};
    real confidence = 0.0_r;
    real omegaDegPerSec = 0.0_r;
    real driveVolts = 0.0_r;
    bool brainInhibits = false;       // disabled or e-stopped
};

class TofArray {
    public:
        TofArray();

        // Reset all devices before creating the shared I2C bus.
        esp_err_t holdAllInReset();

        // Enumerate individually, then configure and stagger starts.
        esp_err_t begin();

        // Poll one sensor; returns fresh evidence valid until the next poll.
        const gflib::SensorReading* poll(const GateContext& ctx);

        const SensorStatus& status(int i) const { return status_[i]; }
        int presentCount() const;

    private:
        // Reduce one record to a projected median or rejection.
        Reject reduce(int i, const RawResult& r, const GateContext& ctx,
                      real& medianInches);

        // Wall incidence in degrees, or negative when outside the field.
        real incidenceDeg(const gflib::Pose& at, const gflib::SensorMount& m) const;

        Tmf8821 dev_[cfg::kTofCount];
        SensorStatus status_[cfg::kTofCount];
        gflib::SensorReading readings_[cfg::kTofCount];

        i2c_master_bus_handle_t bus_ = nullptr;

        // Counter-based scheduling avoids clock drift.
        int slot_ = 0;

        // Tells a fresh record from a re-read.
        uint8_t lastTid_[cfg::kTofCount] = {};
        bool haveTid_[cfg::kTofCount] = {};

        real halfW_ = 72.0_r;
        real halfH_ = 72.0_r;
        real gateMinConfidence_ = 0.3_r;
};

}  // namespace tof
