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
#include "gflib/posesource.hpp"
#include "hal_esp32.hpp"
#include "pod_config.hpp"

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

uint16_t buildFlags(uint32_t nowMs) {
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

    // kMclConverged stays clear until Stage C. Confidence is 1 because dead
    return f;
}

// Returns the moment the frame was handed to the UART, or 0 if none was.
int64_t report(uint32_t nowMs, uint16_t flags) {
    const gflib::Pose p = g_odom.getPose();
    const gflib::Velocity v = g_odom.getVelocity();

    gflib::PoseReport rep;

    // The sender's clock, captured before the sensors were read
    rep.timestampMs = nowMs;

    rep.xInches = p.x;
    rep.yInches = p.y;
    rep.thetaDegrees = p.thetaDeg;
    rep.confidence = 1.0f;
    rep.flags = flags;

    rep.odomXInches = p.x;
    rep.odomYInches = p.y;
    rep.odomThetaDegrees = p.thetaDeg;

    rep.bootId = g_bootId;
    rep.vxInchesPerSec = v.vx;
    rep.vyInchesPerSec = v.vy;
    rep.omegaDegPerSec = v.omegaDegPerSec;

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
void listen(uint32_t nowMs, int64_t txStartUs) {
    // A dropped send provokes no reply, so the window still runs
    const bool timing = txStartUs != 0;
    const int64_t windowStartUs = timing ? txStartUs : esp_timer_get_time();
    const int64_t deadlineUs = windowStartUs + cfg::kRxWindowUs;
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

    g_loop.reset();
    g_rtt.reset();
}

}  // namespace

extern "C" void app_main(void) {
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

    ESP_LOGI(kTag, "ready");

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

        const uint16_t flags = buildFlags(nowMs);

        const int64_t txStartUs = report(nowMs, flags);

        // The Brain replies on decoding that frame, so the wire is quiet
        // until roughly 3-5ms from now. Listen through
        listen(nowMs, txStartUs);

        g_loop.addWork(esp_timer_get_time() - tickUs);

        // Printed last, inside the slack that is left. At 921600 baud a line
        // of this length is well under a millisecond, so it lands before
        // vTaskDelayUntil is due and the period stays flat.
        if (g_loop.ticks >= 100) telemetry();

        vTaskDelayUntil(&last, pdMS_TO_TICKS(cfg::kLoopPeriodMs));
    }
}
