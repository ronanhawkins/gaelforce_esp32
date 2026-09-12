#include "tof_array.hpp"

#include <cinttypes>
#include <cmath>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gflib/util.hpp"

namespace tof {

namespace {

const char* kTag = "tofarray";

constexpr int kZonesUsed = 4;

// Require corroboration from at least two zones.
constexpr int kMinValidZones = 2;

// Average the middle pair for even counts.
real medianOf(real* v, int n) {
    for (int i = 1; i < n; ++i) {
        const real key = v[i];
        int j = i - 1;
        while (j >= 0 && v[j] > key) { v[j + 1] = v[j]; --j; }
        v[j + 1] = key;
    }
    if (n <= 0) return 0.0_r;
    if (n & 1) return v[n / 2];
    return 0.5_r * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace

TofArray::TofArray()
    : dev_{{cfg::kTofEnPins[0], cfg::kTofAddrs[0]},
           {cfg::kTofEnPins[1], cfg::kTofAddrs[1]},
           {cfg::kTofEnPins[2], cfg::kTofAddrs[2]},
           {cfg::kTofEnPins[3], cfg::kTofAddrs[3]}} {
    for (int i = 0; i < cfg::kTofCount; ++i) {
        readings_[i].mountIndex = i;
        readings_[i].valid = false;
    }
}

esp_err_t TofArray::holdAllInReset() {
    esp_err_t first = ESP_OK;
    for (int i = 0; i < cfg::kTofCount; ++i) {
        const esp_err_t e = dev_[i].holdReset();
        if (e != ESP_OK && first == ESP_OK) first = e;
    }
    return first;
}

esp_err_t TofArray::begin() {
    const int64_t startUs = esp_timer_get_time();
    const gflib::MclConfig mc = cfg::makeMcl();
    halfW_ = mc.fieldHalfWidthInches;
    halfH_ = mc.fieldHalfHeightInches;
    gateMinConfidence_ = mc.gateMinConfidence;

    i2c_master_bus_config_t bc = {};
    bc.i2c_port = I2C_NUM_0;
    bc.sda_io_num = static_cast<gpio_num_t>(cfg::kI2cSdaPin);
    bc.scl_io_num = static_cast<gpio_num_t>(cfg::kI2cSclPin);
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;

    // The only pull-ups are the breakouts' own, so bus resistance is theirs
    // divided by however many jumpers are closed, and that count sets the rise
    // time kTofRunHz needs. The internal ~45k cannot drive I2C at any speed.
    bc.flags.enable_internal_pullup = false;

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bc, &bus_), kTag, "i2c bus");

    // Enumerate one at a time while devices share address 0x41.
    for (int i = 0; i < cfg::kTofCount; ++i) {
        const esp_err_t e = dev_[i].bringUp(bus_, cfg::kTofEnumHz);
        if (e != ESP_OK) {
            // Keep failed devices in reset to prevent address collisions.
            dev_[i].holdReset();
            ESP_LOGE(kTag, "sensor %d (EN gpio %d) did not enumerate: %s", i,
                     cfg::kTofEnPins[i], esp_err_to_name(e));
            continue;
        }
        ESP_LOGI(kTag, "sensor %d up at 0x%02X (app minor 0x%02X, calib 0x%02X)", i,
                 dev_[i].address(), dev_[i].appMinorVersion(), dev_[i].calibrationStatus());
    }

    for (int i = 0; i < cfg::kTofCount; ++i) {
        status_[i].present = dev_[i].present();
        status_[i].address = dev_[i].address();
        status_[i].appMinor = dev_[i].appMinorVersion();
        status_[i].calibrationStatus = dev_[i].calibrationStatus();
        status_[i].lastReject = dev_[i].present() ? Reject::None : Reject::Absent;

        if (!dev_[i].present()) continue;

        // Do not expose devices that fail configuration or startup.
        const esp_err_t setup = [&] {
            esp_err_t e = dev_[i].setSclHz(cfg::kTofRunHz);
            if (e != ESP_OK) return e;

            // The mask must land before spad_map_id selects it, or the part
            // configures a user-defined map with nothing behind it.
            if (cfg::kTofSpadMap >= 14) {
                e = dev_[i].downloadSpadMask(cfg::kTofSpadMaskBlob,
                                             sizeof(cfg::kTofSpadMaskBlob));
                if (e != ESP_OK) return e;
            }
            e = dev_[i].configure(cfg::kTofPeriodMs, cfg::kTofKiloIterations,
                                  cfg::kTofSpadMap, 6);
            if (e != ESP_OK) return e;
            return dev_[i].startMeasuring();
        }();

        if (setup != ESP_OK) {
            ESP_LOGE(kTag, "sensor %d enumerated at 0x%02X but would not start: %s",
                     i, dev_[i].address(), esp_err_to_name(setup));
            dev_[i].holdReset();
            status_[i].present = false;
            status_[i].address = 0;
            status_[i].lastReject = Reject::Absent;
            continue;
        }

        if (dev_[i].calibrationStatus() != 0x00) {
            // Expected without factory calibration.
            ESP_LOGW(kTag, "sensor %d CALIBRATION_STATUS 0x%02X -- default calibration",
                     i, dev_[i].calibrationStatus());
        }

        // Stagger free-running publications by one tick.
        vTaskDelay(pdMS_TO_TICKS(cfg::kTofStaggerMs));
    }

    // Report the mandatory boot-time firmware load cost.
    ESP_LOGI(kTag, "%d of %d sensors present, enumeration took %lld ms",
             presentCount(), cfg::kTofCount, (esp_timer_get_time() - startUs) / 1000);
    return ESP_OK;
}

int TofArray::presentCount() const {
    int n = 0;
    for (int i = 0; i < cfg::kTofCount; ++i) if (status_[i].present) ++n;
    return n;
}

real TofArray::incidenceDeg(const gflib::Pose& at, const gflib::SensorMount& m) const {
    // Match Mcl::raycast coordinates.
    const real t = static_cast<real>(at.thetaDeg) * gflib::kDegToRad;
    const real st = std::sin(t), ct = std::cos(t);

    const real ox = static_cast<real>(at.x) + m.yInches * st + m.xInches * ct;
    const real oy = static_cast<real>(at.y) + m.yInches * ct - m.xInches * st;
    if (std::fabs(ox) > halfW_ || std::fabs(oy) > halfH_) return -1.0_r;

    const real b = (static_cast<real>(at.thetaDeg) + m.bearingDeg) * gflib::kDegToRad;
    const real dx = std::sin(b), dy = std::cos(b);

    // Nearest hit using the same rule as raycast.
    real tx = 1e30_r, ty = 1e30_r;
    const real kEps = 1e-9_r;
    if (dx > kEps)       tx = ( halfW_ - ox) / dx;
    else if (dx < -kEps) tx = (-halfW_ - ox) / dx;
    if (dy > kEps)       ty = ( halfH_ - oy) / dy;
    else if (dy < -kEps) ty = (-halfH_ - oy) / dy;

    const real component = (tx < ty) ? std::fabs(dx) : std::fabs(dy);
    return std::acos(gflib::clamp(component, 0.0_r, 1.0_r)) * gflib::kRadToDeg;
}

Reject TofArray::reduce(int i, const RawResult& r, const GateContext& ctx,
                        real& rawInches, real& lagInches) {
    SensorStatus& s = status_[i];

    // Capture raw telemetry before applying gates.
    real proj[kZonesUsed];
    int n = 0;
    for (int z = 0; z < kZonesUsed; ++z) {
        const int zone = cfg::kTofZones[z];
        const ZoneResult& zr = r.zones[zone - 1];

        s.lastZoneMm[z] = zr.distanceMm;
        s.lastZoneConf[z] = zr.confidence;

        // Confidence zero means no target.
        if (zr.confidence == 0 || zr.distanceMm == 0) continue;

        const real inches = static_cast<real>(zr.distanceMm) / 25.4_r;

        // Project off-axis zones onto the boresight.
        proj[n++] = inches * std::cos(cfg::kTofZoneOffAxisDeg[z] * gflib::kDegToRad);
    }

    // Before the motion gate: an empty record is a NoReturn whatever the robot
    // was doing, and calling it Motion hides empty records taken rolling.
    if (n < kMinValidZones) return Reject::NoReturn;

    const real rawMed = medianOf(proj, n);
    rawInches = rawMed;

    // Reject wall edges and partial occlusions. The tolerance scales because an
    // oblique wall's spread grows with range; a fixed one only passed normals.
    const real tol = (cfg::kTofZoneSpreadFrac * rawMed > cfg::kTofZoneSpreadFloorIn)
                         ? cfg::kTofZoneSpreadFrac * rawMed
                         : cfg::kTofZoneSpreadFloorIn;
    for (int k = 0; k < n; ++k) {
        if (std::fabs(proj[k] - rawMed) > tol) return Reject::Dispersion;
    }

    // Both ask what the optics physically did, so they test the RAW median. Lag
    // compensation is a statement about time, not about what the sensor saw.
    if (rawMed > cfg::kTofTrustRadiusIn) return Reject::OutOfRange;

    // Past the knee every zone grazes the same floor and they AGREE, so the
    // spread gate cannot see it. This is the only gate that catches it.
    if (rawMed > cfg::kTofTrustRadiusIn - cfg::kTofSaturationMarginIn) {
        return Reject::Saturated;
    }

    // Last: the gates above describe what the optics did, so their counters
    // stay meaningful while driving. This one is only about trusting it.
    if (ctx.brainInhibits ||
        std::fabs(ctx.omegaDegPerSec) > cfg::kTofMaxYawDegPerSec ||
        std::fabs(ctx.driveVolts) > cfg::kTofMaxDriveVolts) {
        return Reject::Motion;
    }

    // Close the gap: the record describes where the robot was kTofLagMs ago.
    const real bRad = (static_cast<real>(ctx.estimate.thetaDeg) +
                       cfg::kTofMounts[i].bearingDeg) * gflib::kDegToRad;
    const real closingInPerSec =
        ctx.vxInPerSec * std::sin(bRad) + ctx.vyInPerSec * std::cos(bRad);

    lagInches = -closingInPerSec * (cfg::kTofLagMs / 1000.0_r);

    // Apply the geometric gate only when the estimate is trusted.
    if (ctx.confidence >= gateMinConfidence_) {
        const real inc = incidenceDeg(ctx.estimate, cfg::kTofMounts[i]);
        if (inc < 0.0_r) {
            // Count incidence checks skipped outside the field.
            ++s.incidenceUncomputable;
        } else if (inc > cfg::kTofMaxIncidenceDeg) {
            return Reject::Oblique;
        }
    }

    return Reject::None;
}

const gflib::SensorReading* TofArray::poll(const GateContext& ctx) {
    // INT_STATUS is one byte, the result is kResultBytes. Checking all four and
    // reading only the ready one drops the time to notice a publication from a
    // full round-robin to one tick. kTofStaggerMs keeps at most one ready.
    int i = -1;
    for (int n = 0; n < cfg::kTofCount; ++n) {
        const int c = (slot_ + n) % cfg::kTofCount;
        if (!status_[c].present) continue;
        if (dev_[c].resultReady()) { i = c; break; }
    }

    // Rotate the start so a tie never starves the same sensor.
    slot_ = (slot_ + 1) % cfg::kTofCount;
    if (i < 0) return nullptr;

    // Consume each reading once.
    readings_[i].valid = false;

    RawResult r;
    if (!dev_[i].readResult(r)) {
        status_[i].busErrors = dev_[i].busErrors();
        return nullptr;
    }

    // Ignore republished records.
    if (haveTid_[i] && r.tid == lastTid_[i]) return nullptr;
    lastTid_[i] = r.tid;
    haveTid_[i] = true;

    SensorStatus& s = status_[i];
    if (s.results == 0) {
        // Measure startup latency under the active configuration.
        s.firstResultMs = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        ESP_LOGI(kTag, "tof%d first result at %" PRIu32 " ms since boot", i,
                 s.firstResultMs);
    }
    ++s.results;
    s.busErrors = dev_[i].busErrors();

    // Raw is what the optics measured; lag is what MCL adds. Kept apart so the
    // column does not mean two things on accept and on reject.
    real raw = 0.0_r, lag = 0.0_r;
    const Reject why = reduce(i, r, ctx, raw, lag);

    s.lastReject = why;
    s.lastMedianInches = raw;
    s.lastLagInches = lag;
    s.lastValid = (why == Reject::None);
    ++s.rejects[static_cast<int>(why)];

    if (why != Reject::None) return nullptr;
    ++s.accepted;

    readings_[i].mountIndex = i;

    // MCL gets the lag-corrected value; status keeps the raw one.
    readings_[i].distanceInches = raw + lag;
    readings_[i].valid = true;
    return &readings_[i];
}

}  // namespace tof
