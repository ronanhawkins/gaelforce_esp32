#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "boot_id.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gflib/link.hpp"
#include "gflib/mcl.hpp"
#include "gflib/posesource.hpp"
#include "hal_esp32.hpp"
#include "pod_config.hpp"
#include "tof_array.hpp"

namespace {

const char* kTag = "pod";

// Single task throughout. This runs in app_main itself

hal::EspClock g_clock;
//configure 2 pcnt encoders, 1 uart rvc imu and the uart rs485 link
hal::PcntEncoder g_vert(cfg::kVertEncAPin, cfg::kVertEncBPin, cfg::kVertReversed);
hal::PcntEncoder g_horiz(cfg::kHorizEncAPin, cfg::kHorizEncBPin, cfg::kHorizReversed);
hal::RvcImu g_imu(UART_NUM_1, cfg::kImuRxPin);
hal::Rs485Stream g_link(UART_NUM_2, cfg::kRs485TxPin, cfg::kRs485RxPin, cfg::kRs485DePin);

gflib::OdomPoseSource g_odom(g_vert, g_horiz, g_imu, g_clock, cfg::makeOdom());

// MCL cloud and correcting sensors.
gflib::Mcl g_mcl;
tof::TofArray g_tof;

// State for rate-limited divergence recovery.
uint32_t g_lastReseedMs = 0;
uint32_t g_mclReseeds = 0;
bool g_haveReseeded = false;

// Separate MCL and I2C timing statistics.
int64_t g_mclMaxUs = 0;
int64_t g_mclSumUs = 0;
int64_t g_tofMaxUs = 0;
int64_t g_tofSumUs = 0;
uint32_t g_mclTicks = 0;

gflib::FrameWriter g_writer;
gflib::FrameParser g_parser;

uint16_t g_bootId = 0;

// Last status the Brain sent, and the local millis it landed at.
gflib::BrainStatus g_status{};
bool g_haveStatus = false;
uint32_t g_statusMs = 0;

// Set when a PoseSet lands, cleared by the answering report
bool g_poseResetPending = false;

// Pose frames the UART could not take whole. Should stay at zero
// If higher baud is wrong or brain is transmitting over the link
uint32_t g_droppedSends = 0;

// instrumentation

struct LoopStats {
    int64_t periodMinUs = INT64_MAX;
    int64_t periodMaxUs = 0;
    int64_t periodSumUs = 0;
    int64_t workMaxUs = 0;
    uint32_t ticks = 0;

    void addPeriod(int64_t us) {
        if (us < periodMinUs) periodMinUs = us;
        if (us > periodMaxUs) periodMaxUs = us;
        periodSumUs += us;
        ++ticks;
    }
    void addWork(int64_t us) { if (us > workMaxUs) workMaxUs = us; }
    void reset() { *this = LoopStats{}; }
};

LoopStats g_loop;

// Round trip, pod clock only, the moment the pose frame is handed to the UART
// to the moment the reply it provoked finishes decoding.
struct RttStats {
    int64_t minUs = INT64_MAX;
    int64_t maxUs = 0;
    int64_t sumUs = 0;
    uint32_t samples = 0;

    void add(int64_t us) {
        if (us < minUs) minUs = us;
        if (us > maxUs) maxUs = us;
        sumUs += us;
        ++samples;
    }
    void reset() { *this = RttStats{}; }
};

RttStats g_rtt;

// Actual on-wire sizes, not sizeof() of the structs
size_t g_lastPoseFrameBytes = 0;
size_t g_lastStatusFrameBytes = 0;

// Dispatches everything the parser can resolve out of what has been pushed.
// Returns true if a BrainStatus was among it, which is what closes the
// round-trip measurement.
bool drainParser(uint32_t nowMs) {
    bool sawStatus = false;

    gflib::DecodeResult r;
    while (g_parser.poll(r)) {
        if (r.err != gflib::LinkError::None) continue;

        switch (r.type) {
            case gflib::MsgType::BrainStatus: {
                gflib::BrainStatus s;
                if (gflib::decodeBrainStatus(r, s)) {
                    g_lastStatusFrameBytes =
                        gflib::kLinkHeaderBytes + r.payloadLen + gflib::kLinkCrcBytes;
                    g_status = s;
                    g_haveStatus = true;
                    g_statusMs = nowMs;
                    sawStatus = true;
                }
                break;
            }
            case gflib::MsgType::PoseSet: {
                gflib::PoseSet ps;
                if (gflib::decodePoseSet(r, ps)) {
                    gflib::Pose p;
                    p.x = ps.xInches;
                    p.y = ps.yInches;
                    p.thetaDeg = ps.thetaDegrees;

                    g_odom.setPose(p);

                    // Keep the cloud aligned with the odometry reset.
                    g_mcl.reseed(p, cfg::kMclSeedPosSigmaIn, cfg::kMclSeedHeadingSigmaDeg);

                    g_poseResetPending = true;

                    ESP_LOGI(kTag, "PoseSet -> %.2f %.2f %.2f", ps.xInches, ps.yInches,
                             ps.thetaDegrees);
                }
                break;
            }
            default:
                break;
        }
    }
    return sawStatus;
}

uint16_t buildFlags(uint32_t nowMs, double confidence) {
    uint16_t f = 0;

    if (g_imu.ok(nowMs)) f |= gflib::PoseFlags::kImuOk;

    // OdomPoseSource drops implausible readings silently and exposes no count
    const gflib::OdomSourceConfig& oc = g_odom.config();
    const double maxVert = oc.maxTravelInchesPerTick / oc.odom.vertInchesPerCount;
    const double maxHoriz = oc.maxTravelInchesPerTick / oc.odom.horizInchesPerCount;

    if (std::fabs(g_vert.deltaCounts()) <= maxVert) f |= gflib::PoseFlags::kVertPodOk;
    if (std::fabs(g_horiz.deltaCounts()) <= maxHoriz) f |= gflib::PoseFlags::kHorizPodOk;

    if (!g_haveStatus || gflib::linkIsStale(nowMs, g_statusMs, cfg::kBrainStatusTimeoutMs)) {
        f |= gflib::PoseFlags::kLinkDegraded;
    }

    // Deliberately not cleared here. The Brain refuses to drive until it
    // sees this echo
    if (g_poseResetPending) f |= gflib::PoseFlags::kPoseReset;

    // Reuse the filter's gating-confidence threshold.
    if (!g_mcl.diverged() && confidence >= g_mcl.config().gateMinConfidence) {
        f |= gflib::PoseFlags::kMclConverged;
    }

    return f;
}

// Returns the moment the frame was handed to the UART, or 0 if none was.
int64_t report(uint32_t nowMs, uint16_t flags, const gflib::Pose& mcl, double confidence) {
    const gflib::Pose p = g_odom.getPose();
    const gflib::Velocity v = g_odom.getVelocity();

    gflib::PoseReport rep;

    // The sender's clock, captured before the sensors were read
    rep.timestampMs = nowMs;

    rep.xInches = static_cast<float>(mcl.x);
    rep.yInches = static_cast<float>(mcl.y);
    rep.thetaDegrees = static_cast<float>(mcl.thetaDeg);
    rep.confidence = static_cast<float>(confidence);
    rep.flags = flags;

    rep.odomXInches = static_cast<float>(p.x);
    rep.odomYInches = static_cast<float>(p.y);
    rep.odomThetaDegrees = static_cast<float>(p.thetaDeg);

    rep.bootId = g_bootId;
    // MCL resampling makes pose-derived velocity unstable.
    rep.vxInchesPerSec = static_cast<float>(v.vx);
    rep.vyInchesPerSec = static_cast<float>(v.vy);
    rep.omegaDegPerSec = static_cast<float>(v.omegaDegPerSec);

    uint8_t frame[gflib::kLinkMaxFrame];
    const size_t n = g_writer.poseReport(rep, frame, sizeof(frame));
    if (n == 0) return 0;

    const int64_t txStartUs = esp_timer_get_time();

    if (g_link.write(frame, n) != n) {
        // Nothing went out, so the reset echo has not been delivered
        ++g_droppedSends;
        return 0;
    }

    g_lastPoseFrameBytes = n;
    g_poseResetPending = false;
    return txStartUs;
}

// Empties whatever arrived between windows.
void drainPending(uint32_t nowMs) {
    uint8_t chunk[128];
    for (;;) {
        const size_t n = g_link.read(chunk, sizeof(chunk));
        if (n == 0) break;

        size_t pushed = 0;
        int stalls = 0;
        while (pushed < n) {
            const size_t took = g_parser.push(chunk + pushed, n - pushed);
            if (took == 0) {
                if (++stalls > 8) break;
                drainParser(nowMs);
                continue;
            }
            stalls = 0;
            pushed += took;
            drainParser(nowMs);
        }
    }
}

// Spends the tick's slack listening instead of sleeping. Returns when the
// reply has been decoded or the window closes
void listen(uint32_t nowMs, int64_t txStartUs, int64_t windowUs) {
    // A dropped send provokes no reply, so the window still runs
    const bool timing = txStartUs != 0;
    const int64_t windowStartUs = timing ? txStartUs : esp_timer_get_time();
    const int64_t deadlineUs = windowStartUs + windowUs;
    uint8_t chunk[128];

    for (;;) {
        const int64_t remainingUs = deadlineUs - esp_timer_get_time();
        if (remainingUs <= 0) break;

        const size_t n = g_link.readBlocking(chunk, sizeof(chunk), remainingUs);
        if (n == 0) break;   // window closed with nothing on the wire

        int64_t rttUs = 0;
        size_t pushed = 0;
        int stalls = 0;

        // Every byte read has to reach the parser before this returns.
        while (pushed < n) {
            const size_t took = g_parser.push(chunk + pushed, n - pushed);
            if (took == 0) {
                // Buffer full because a frame is waiting to be polled out.
                if (++stalls > 8) break;
                drainParser(nowMs);
                continue;
            }
            stalls = 0;
            pushed += took;

            if (drainParser(nowMs) && timing && rttUs == 0) {
                rttUs = esp_timer_get_time() - txStartUs;
            }
        }

        if (rttUs != 0) {
            g_rtt.add(rttUs);
            return;
        }
    }
}

void telemetry() {
    if (g_loop.ticks == 0) return;

    const gflib::LinkStats& s = g_parser.stats();
    const gflib::Pose p = g_odom.getPose();

    const double meanPeriodUs = static_cast<double>(g_loop.periodSumUs) / g_loop.ticks;

    ESP_LOGI(kTag,
             "pose %.2f %.2f %.2f | enc %.0f %.0f hdg %.2f imu %" PRIu32 "/%" PRIu32
             " | loop %.2f/%.2f/%.2f ms work %.2f",
             p.x, p.y, p.thetaDeg, g_vert.getCounts(), g_horiz.getCounts(),
             g_imu.getHeadingDeg(), g_imu.frameCount(), g_imu.checksumErrors(),
             g_loop.periodMinUs / 1000.0, meanPeriodUs / 1000.0, g_loop.periodMaxUs / 1000.0,
             g_loop.workMaxUs / 1000.0);

    if (g_rtt.samples > 0) {
        // Air time is exact at this baud and must come off before the
        // remainder is halved
        const double poseAirMs = cfg::airTimeMs(g_lastPoseFrameBytes);
        const double statusAirMs = cfg::airTimeMs(g_lastStatusFrameBytes);

        const double meanRttMs = (static_cast<double>(g_rtt.sumUs) / g_rtt.samples) / 1000.0;
        const double unknownMs = meanRttMs - poseAirMs - statusAirMs;
        const double oneWayMs = poseAirMs + unknownMs / 2.0;

        ESP_LOGI(kTag,
                 "rtt %.2f/%.2f/%.2f ms n=%" PRIu32
                 " | air pose %.2f status %.2f | transit %.2f ms",
                 g_rtt.minUs / 1000.0, meanRttMs, g_rtt.maxUs / 1000.0, g_rtt.samples,
                 poseAirMs, statusAirMs, oneWayMs);
    } else {
        ESP_LOGW(kTag, "rtt: no BrainStatus replies this window");
    }

    ESP_LOGI(kTag,
             "link crc %" PRIu32 " resync %" PRIu32 " dropped %" PRIu32 " ooo %" PRIu32
             " ver %" PRIu32 " len %" PRIu32 " unk %" PRIu32 " frames %" PRIu32,
             s.crcErrors, s.resyncBytes, s.droppedFrames, s.outOfOrderFrames,
             s.versionMismatches, s.lengthErrors, s.unknownTypes, s.framesDecoded);

    if (g_droppedSends > 0) ESP_LOGW(kTag, "dropped sends %" PRIu32, g_droppedSends);

    // MCL pose, spread, and divergence diagnostics.
    const gflib::Pose m = g_mcl.estimate();
    ESP_LOGI(kTag,
             "mcl %.2f %.2f %.2f | conf %.2f spread %.2f hdg-sd %.2f ess %.0f/%d"
             " | gated %" PRIu32 " diverged %" PRIu32 " reseeds %" PRIu32 "%s",
             m.x, m.y, m.thetaDeg, g_mcl.confidence(), g_mcl.positionStdDevInches(),
             g_mcl.headingStdDevDeg(), g_mcl.effectiveSampleSize(), g_mcl.particleCount(),
             g_mcl.gatedReadings(), g_mcl.divergences(), g_mclReseeds,
             g_mcl.diverged() ? " DIVERGED" : "");

    if (g_mclTicks > 0) {
        ESP_LOGI(kTag,
                 "cost mcl %.2f/%.2f ms | tof i2c %.2f/%.2f ms  mean/max over %" PRIu32
                 " ticks (%d particles)",
                 (static_cast<double>(g_mclSumUs) / g_mclTicks) / 1000.0, g_mclMaxUs / 1000.0,
                 (static_cast<double>(g_tofSumUs) / g_mclTicks) / 1000.0, g_tofMaxUs / 1000.0,
                 g_mclTicks, g_mcl.particleCount());
    }
    g_mclMaxUs = 0;
    g_mclSumUs = 0;
    g_tofMaxUs = 0;
    g_tofSumUs = 0;
    g_mclTicks = 0;

    // Per-sensor ToF diagnostics expose mount errors.
    static const char* kRejectName[] = {"ok", "absent", "noret", "range",
                                        "disp", "obliq", "motion"};
    for (int i = 0; i < cfg::kTofCount; ++i) {
        const tof::SensorStatus& t = g_tof.status(i);
        if (!t.present) {
            ESP_LOGW(kTag, "tof%d ABSENT (never enumerated)", i);
            continue;
        }
        ESP_LOGI(kTag,
                 "tof%d @0x%02X cal 0x%02X t1st %" PRIu32 " ms | zones %4u/%3u %4u/%3u %4u/%3u %4u/%3u mm/conf"
                 " | med %6.2f in %s | n %" PRIu32 " ok %" PRIu32
                 " noret %" PRIu32 " range %" PRIu32 " disp %" PRIu32
                 " obliq %" PRIu32 " motion %" PRIu32 " nogate %" PRIu32 " bus %" PRIu32,
                 i, t.address, t.calibrationStatus, t.firstResultMs,
                 t.lastZoneMm[0], t.lastZoneConf[0], t.lastZoneMm[1], t.lastZoneConf[1],
                 t.lastZoneMm[2], t.lastZoneConf[2], t.lastZoneMm[3], t.lastZoneConf[3],
                 t.lastMedianInches, kRejectName[static_cast<int>(t.lastReject)],
                 t.results, t.accepted,
                 t.rejects[static_cast<int>(tof::Reject::NoReturn)],
                 t.rejects[static_cast<int>(tof::Reject::OutOfRange)],
                 t.rejects[static_cast<int>(tof::Reject::Dispersion)],
                 t.rejects[static_cast<int>(tof::Reject::Oblique)],
                 t.rejects[static_cast<int>(tof::Reject::Motion)],
                 t.incidenceUncomputable, t.busErrors);
    }

    g_loop.reset();
    g_rtt.reset();
}

}  // namespace

extern "C" void app_main(void) {
    // Hold every sensor low before creating the shared I2C bus.
    ESP_ERROR_CHECK(g_tof.holdAllInReset());

    g_bootId = bootid::next();
    ESP_LOGI(kTag, "gaelforce pod, bootId %u, link %" PRIu32 " baud", g_bootId,
             gflib::kLinkBaud);

    const gflib::OdomSourceConfig startupCfg = cfg::makeOdom();
    if (!(startupCfg.odom.vertInchesPerCount > 0.0f) ||
        !(startupCfg.odom.horizInchesPerCount > 0.0f)) {
        ESP_LOGE(kTag, "inchesPerCount is not positive -- check kTrackingWheelDiaIn");
        abort();
    }

    ESP_ERROR_CHECK(g_vert.begin());
    ESP_ERROR_CHECK(g_horiz.begin());
    ESP_ERROR_CHECK(g_imu.begin());
    ESP_ERROR_CHECK(g_link.begin());

    // Let a few RVC frames land before the baseline is taken
    vTaskDelay(pdMS_TO_TICKS(200));
    g_vert.sample();
    g_horiz.sample();
    g_imu.sample();

    // Seeds the sensor baselines from the robot as it stands.
    g_odom.begin(0);

    // Missing sensors degrade operation but are not fatal.
    ESP_ERROR_CHECK_WITHOUT_ABORT(g_tof.begin());

    // Seed from odometry; bootId makes the RNG seed reproducible.
    const gflib::MclConfig mclCfg = cfg::makeMcl();
    g_mcl.init(mclCfg, g_odom.getPose(), cfg::kMclSeedPosSigmaIn,
               cfg::kMclSeedHeadingSigmaDeg, g_bootId);

    // Warn every boot until the interior field size is confirmed.
    ESP_LOGW(kTag,
             "field interior %.1f x %.1f in (PLACEHOLDER -- confirm against the "
             "game manual and a tape measure)",
             2.0 * mclCfg.fieldHalfWidthInches, 2.0 * mclCfg.fieldHalfHeightInches);

    ESP_LOGI(kTag, "ready, %d/%d tof sensors", g_tof.presentCount(), cfg::kTofCount);

    TickType_t last = xTaskGetTickCount();
    int64_t prevTickUs = esp_timer_get_time();

    for (;;) {
        const int64_t tickUs = esp_timer_get_time();
        g_loop.addPeriod(tickUs - prevTickUs);
        prevTickUs = tickUs;

        // Captured once, before any sensor is touched, used to
        // compute the pose and to stamp the report.
        const uint32_t nowMs = g_clock.millisNow();

        g_vert.sample();
        g_horiz.sample();
        g_imu.sample();

        // Between the latch and the integration: see drainPending.
        drainPending(nowMs);

        g_odom.update();

        // Predict from the deltas odometry actually accepted.
        const int64_t mclStartUs = esp_timer_get_time();

        const gflib::OdomPoseSource::IntegratedDeltas d = g_odom.lastDeltas();
        g_mcl.predict(d.vertCounts, d.horizCounts, d.thetaDeg, g_odom.config().odom);

        // Traverse the cloud once for gating, flags, and reporting.
        const gflib::Pose mclPose = g_mcl.estimate();
        const double confidence = g_mcl.confidence();

        tof::GateContext gate;
        gate.estimate = mclPose;
        gate.confidence = static_cast<gflib::real>(confidence);
        gate.omegaDegPerSec = g_odom.getVelocity().omegaDegPerSec;

        // Treat stale drive voltage as unknown; yaw remains measured.
        const bool statusFresh =
            g_haveStatus && !gflib::linkIsStale(nowMs, g_statusMs, cfg::kBrainStatusTimeoutMs);
        if (statusFresh) {
            const double l = std::fabs(g_status.leftVolts);
            const double r = std::fabs(g_status.rightVolts);
            gate.driveVolts = static_cast<float>(l > r ? l : r);
            gate.brainInhibits =
                (g_status.flags & (gflib::BrainFlags::kDisabled | gflib::BrainFlags::kEstop)) != 0;
        }

        const int64_t mclPredictUs = esp_timer_get_time() - mclStartUs;

        // Poll one sensor per tick and time I2C separately.
        const int64_t tofStartUs = esp_timer_get_time();
        const gflib::SensorReading* fresh = g_tof.poll(gate);
        const int64_t tofUs = esp_timer_get_time() - tofStartUs;

        const int64_t mclResumeUs = esp_timer_get_time();
        if (fresh != nullptr) {
            g_mcl.update(cfg::kTofMounts, cfg::kTofCount, fresh, 1);
        }

        // Recover latched divergence from odometry, with rate limiting.
        if (g_mcl.diverged() &&
            (!g_haveReseeded ||
             gflib::linkElapsedMs(nowMs, g_lastReseedMs) >= cfg::kMclReseedMinIntervalMs)) {
            g_mcl.reseed(g_odom.getPose(), cfg::kMclSeedPosSigmaIn,
                         cfg::kMclSeedHeadingSigmaDeg);
            g_lastReseedMs = nowMs;
            g_haveReseeded = true;
            ++g_mclReseeds;
        }

        const int64_t mclUs = mclPredictUs + (esp_timer_get_time() - mclResumeUs);
        if (mclUs > g_mclMaxUs) g_mclMaxUs = mclUs;
        if (tofUs > g_tofMaxUs) g_tofMaxUs = tofUs;
        g_mclSumUs += mclUs;
        g_tofSumUs += tofUs;
        ++g_mclTicks;

        const uint16_t flags = buildFlags(nowMs, confidence);

        const int64_t txStartUs = report(nowMs, flags, mclPose, confidence);

        // Use a short window when no fresh Brain reply is expected.
        listen(nowMs, txStartUs, statusFresh ? cfg::kRxWindowUs : cfg::kRxWindowIdleUs);

        g_loop.addWork(esp_timer_get_time() - tickUs);

        // Printed last, inside the slack that is left. At 921600 baud a line
        // of this length is well under a millisecond, so it lands before
        // vTaskDelayUntil is due and the period stays flat.
        if (g_loop.ticks >= 100) telemetry();

        vTaskDelayUntil(&last, pdMS_TO_TICKS(cfg::kLoopPeriodMs));
    }
}
