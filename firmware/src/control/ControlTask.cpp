#include "control/ControlTask.h"

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <math.h>

#include "Config.h"
#include "PinConfig.h"
#include "est/GravityFilter.h"
#include "imu/FifoDecoder.h"
#include "imu/Lsm6dsv16x.h"
#include "util/Half.h"
#include "util/Snapshot.h"
#include "util/Wrap.h"

namespace {

constexpr uint32_t kIntTimeoutMs    = 3;   // about 3 IMU periods
constexpr uint32_t kWhoAmIPerCycle  = 8;
constexpr int kMaxDrainPasses       = 4;
constexpr float kDegToRad           = 0.017453292519943f;
// IMU timestamp counter: 48 ticks per 960 Hz sample, from the same clock as
// the ODR (so INTERNAL_FREQ_FINE scales both). Checked with `imu`.
constexpr float kTsTicksPerSecond   = 46080.0f;

enum class ReqKind : uint8_t { GameVector, WhoAmI, Zero, PeakStart };
struct Request {
    ReqKind kind;
    uint32_t arg;
    uint32_t id;
};

TaskHandle_t gTask    = nullptr;
QueueHandle_t gQueue  = nullptr;
::Snapshot<ControlTask::Snapshot> gSnap;
::Snapshot<ControlTask::WhoAmIResult> gWho;
volatile int64_t gIntUs   = 0;  // last INT1 time (Stage 5 latency)
float gStartMountPitch    = 0.0f;

void IRAM_ATTR int1Isr(void *)
{
    gIntUs              = esp_timer_get_time();
    BaseType_t woken    = pdFALSE;
    vTaskNotifyGiveFromISR(gTask, &woken);
    portYIELD_FROM_ISR(woken);
}

GravityFilterConfig filterConfig(bool leverArm)
{
    GravityFilterConfig c;
    c.kp            = kFilterKp;
    c.ki            = kFilterKi;
    c.alphaCutoffHz = kFilterAlphaCutoffHz;
    c.leverArm      = leverArm;
    c.rImu          = Vec3(kImuPosM[0], kImuPosM[1], kImuPosM[2]);
    c.dC            = kAxleToRingCentreM;
    c.ringHalfWidth  = kRingHalfWidthM;
    c.ringHalfHeight = kRingHalfHeightM;
    c.ringCutRadius  = 0.5f * kRingCutDiameterM;
    return c;
}

// State owned by the control task.
struct State {
    ControlTask::Snapshot snap{};
    FifoDecoder decoder;
    uint32_t pairsThisDrain = 0;
    bool overrunLatched     = false;

    // Estimator.
    GravityFilter filter{filterConfig(true)};
    GravityFilter filterNoComp{filterConfig(false)};
    ContactMode mode = ContactMode::Wheel;
    float thetaC     = 0.0f;
    float hyst       = 0.0f;
    float mount      = 0.0f;   // mounting offset about y, rad
    float nominalDt  = 0.0f;
    float tickS      = 0.0f;
    bool haveTs      = false;
    uint32_t lastTs  = 0;
    float thetaSflp  = 0.0f;

    // Peak-error window.
    float thetaGyro = 0.0f, thetaGyro0 = 0.0f, noComp0 = 0.0f, sflp0 = 0.0f;

    // WHO_AM_I request in progress.
    uint32_t whoRemaining = 0;
    ControlTask::WhoAmIResult who{};

    void init(float mountPitch)
    {
        mount = mountPitch;
        const float odr = Lsm6dsv16x::kOdrHz * (1.0f + 0.0013f * Lsm6dsv16x::freqFine());
        nominalDt = 1.0f / odr;
        tickS     = 1.0f / (kTsTicksPerSecond * (1.0f + 0.0013f * Lsm6dsv16x::freqFine()));
        const RingShape ring(kRingHalfWidthM, kRingHalfHeightM, kAxleToRingCentreM,
                             0.5f * kRingCutDiameterM);
        thetaC = wheelContactLimit(ring, kAxleToRingCentreM, kWheelRadiusM);
        snap.est.thetaC = thetaC;
        hyst      = kModeHysteresisDeg * kDegToRad;
        snap.est.mountPitch = mount;
    }

    // Sensor axes -> body axes (R_sb), then the mounting offset.
    Vec3 toBody(const float s[3]) const
    {
        float b[3];
        applyAxisMap(kRsb, s, b);
        return rotateY(Vec3(b[0], b[1], b[2]), mount);
    }

    float sampleDt(uint32_t ts)
    {
        float dt = nominalDt;
        if (haveTs) {
            dt = static_cast<float>(ts - lastTs) * tickS;  // wraps correctly
            if (dt < 0.5f * nominalDt || dt > 4.5f * nominalDt) {
                ++snap.imu.dtGlitches;
                dt = nominalDt;
            }
        }
        haveTs = true;
        lastTs = ts;
        return dt;
    }

    void updatePeak(float dt)
    {
        ControlTask::PeakStats &p = snap.peak;
        if (!p.active) return;
        p.seconds += dt;
        thetaGyro += filter.thetaDot() * dt;
        const float swing  = fabsf(wrapPi(thetaGyro - thetaGyro0));
        const float comp   = fabsf(wrapPi(filter.theta() - thetaGyro));
        const float noComp = fabsf(wrapPi(filterNoComp.theta() - thetaGyro) - noComp0);
        const float sflp   = fabsf(wrapPi(thetaSflp - thetaGyro) - sflp0);
        if (swing > p.swing) p.swing = swing;
        if (comp > p.comp) p.comp = comp;
        if (noComp > p.noComp) p.noComp = noComp;
        if (sflp > p.sflp) p.sflp = sflp;
    }

    void startPeak()
    {
        snap.peak        = {};
        snap.peak.active = true;
        thetaGyro = thetaGyro0 = filter.theta();
        noComp0 = wrapPi(filterNoComp.theta() - thetaGyro);
        sflp0   = wrapPi(thetaSflp - thetaGyro);
    }

    // `zero`: theta = 0 now.
    void zero()
    {
        const float d = filter.theta();
        mount = wrapPi(mount + d);
        filter.rotateAboutY(d);
        filterNoComp.rotateAboutY(d);
        snap.est.mountPitch = mount;
    }

    void publishEstimate(float dt)
    {
        ControlTask::Estimate &e = snap.est;
        e.theta       = filter.theta();
        e.thetaDot    = filter.thetaDot();
        e.lambda      = filter.lambda();
        e.psi         = filter.psi();
        e.psiDot      = filter.psiDot();
        e.poseValid   = filter.poseValid();
        e.ringMode    = mode == ContactMode::Ring;
        e.bias[0]     = filter.bias().x;
        e.bias[1]     = filter.bias().y;
        e.bias[2]     = filter.bias().z;
        e.accelTrust  = filter.accelTrust();
        e.thetaNoComp = filterNoComp.theta();
        e.thetaSflp   = thetaSflp;
        e.dt          = dt;
    }

    // FifoDecoder sink.
    void onPair(const ImuPair &p)
    {
        float gs[3], as[3];
        for (int i = 0; i < 3; ++i) {
            gs[i] = p.gyro[i] * (Lsm6dsv16x::kGyroDpsPerLsb * kDegToRad);
            as[i] = p.accel[i] * Lsm6dsv16x::kAccelGPerLsb;
        }
        const Vec3 g = toBody(gs);
        const Vec3 a = toBody(as);
        snap.gyro[0]  = g.x; snap.gyro[1]  = g.y; snap.gyro[2]  = g.z;
        snap.accel[0] = a.x; snap.accel[1] = a.y; snap.accel[2] = a.z;
        snap.imuTs    = p.ts;
        ++snap.imu.pairs;
        ++pairsThisDrain;

        // One estimator update per accel + gyro pair (spec §5.2 step 4).
        // Wheel speed is 0 until the drivers report it (Stage 4).
        const float dt = sampleDt(p.ts);
        GravityInput in;
        in.gyro  = g;
        in.accel = a * GravityFilter::kG;
        in.dt    = dt;
        in.mode  = mode;
        filter.update(in);
        filterNoComp.update(in);
        mode = nextContactMode(mode, filter.theta(), thetaC, hyst);

        updatePeak(dt);
        publishEstimate(dt);
    }
    void onGravity(const int16_t v[3])
    {
        float gs[3];
        for (int i = 0; i < 3; ++i) gs[i] = v[i] * (Lsm6dsv16x::kSflpGravMgPerLsb * 0.001f);
        const Vec3 g = toBody(gs);
        snap.gravity[0] = g.x; snap.gravity[1] = g.y; snap.gravity[2] = g.z;
        thetaSflp = atan2f(-g.x, g.z);  // same convention as the accelerometer
        ++snap.gravityCount;
    }
    void onGame(const uint16_t h[3])
    {
        // Unit quaternion with w >= 0 sent as x, y, z only.
        float q[4];
        q[1]           = halfToFloat(h[0]);
        q[2]           = halfToFloat(h[1]);
        q[3]           = halfToFloat(h[2]);
        const float ww = 1.0f - q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        q[0]           = ww > 0.0f ? sqrtf(ww) : 0.0f;
        applyAxisMapQuat(kRsb, q, snap.quat);
        ++snap.quatCount;
    }
    void onBadTag(uint8_t) { ++snap.imu.badTags; }
    void onUnpaired() { ++snap.imu.unpaired; }
};

void drainFifo(State &s)
{
    s.pairsThisDrain = 0;
    uint8_t buf[Lsm6dsv16x::kMaxWordsPerRead * kFifoWordBytes];

    // Read until the FIFO is empty, so INT1 drops below the watermark and the
    // next sample gives a fresh rising edge.
    for (int pass = 0; pass < kMaxDrainPasses; ++pass) {
        lsm6dsv16x_fifo_status_t st;
        if (!Lsm6dsv16x::readFifoStatus(st)) {
            ++s.snap.imu.spiErrors;
            return;
        }
        if (st.fifo_ovr && !s.overrunLatched) ++s.snap.imu.overruns;
        s.overrunLatched = st.fifo_ovr;

        uint16_t level = st.fifo_level;
        if (level > s.snap.imu.maxLevel) s.snap.imu.maxLevel = level;
        if (level == 0) break;

        while (level > 0) {
            const uint16_t n =
                level < Lsm6dsv16x::kMaxWordsPerRead ? level : Lsm6dsv16x::kMaxWordsPerRead;
            if (!Lsm6dsv16x::readFifoWords(buf, n)) {
                ++s.snap.imu.spiErrors;
                return;
            }
            for (uint16_t i = 0; i < n; ++i) s.decoder.feed(&buf[i * kFifoWordBytes], s);
            level = static_cast<uint16_t>(level - n);
        }
    }
    if (s.pairsThisDrain > 2) ++s.snap.imu.late;
}

void handleRequests(State &s)
{
    Request r;
    while (xQueueReceive(gQueue, &r, 0) == pdTRUE) {
        switch (r.kind) {
            case ReqKind::GameVector:
                if (!Lsm6dsv16x::setGameVectorBatch(r.arg != 0)) ++s.snap.imu.spiErrors;
                break;
            case ReqKind::WhoAmI:
                s.whoRemaining = r.arg;
                s.who          = {};
                s.who.id       = r.id;
                break;
            case ReqKind::Zero:
                s.zero();
                s.snap.zeroAck = r.id;
                break;
            case ReqKind::PeakStart:
                s.startPeak();
                break;
        }
    }

    if (s.whoRemaining > 0) {
        for (uint32_t i = 0; i < kWhoAmIPerCycle && s.whoRemaining > 0; ++i, --s.whoRemaining) {
            uint8_t id = 0;
            if (!Lsm6dsv16x::readWhoAmI(id)) {
                ++s.who.spiErrors;
            } else if (id == LSM6DSV16X_ID) {
                ++s.who.good;
            } else {
                ++s.who.wrong;
                s.who.lastWrong = id;
            }
        }
        if (s.whoRemaining == 0) gWho.write(s.who);
    }
}

void controlTask(void *)
{
    // Set before the ISR can fire: at this priority the task may run before
    // xTaskCreatePinnedToCore() has returned the handle.
    gTask = xTaskGetCurrentTaskHandle();

    // Core 1: the GPIO ISR lands here (spec §8.2 rule 1). If INT1 can't be
    // set up, the 3 ms timeout still drains the FIFO (counted as intTimeouts).
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err == ESP_ERR_INVALID_STATE) err = ESP_OK;  // already installed
    gpio_config_t io = {};
    io.pin_bit_mask  = 1ULL << PIN_IMU_INT1;
    io.mode          = GPIO_MODE_INPUT;
    io.pull_down_en  = GPIO_PULLDOWN_DISABLE;  // push-pull output on the IMU
    io.pull_up_en    = GPIO_PULLUP_DISABLE;
    io.intr_type     = GPIO_INTR_POSEDGE;
    if (err == ESP_OK) err = gpio_config(&io);
    if (err == ESP_OK) err = gpio_isr_handler_add(PIN_IMU_INT1, int1Isr, nullptr);

    static State s;  // internal RAM (.bss), not the stack
    s.init(gStartMountPitch);
    s.snap.int1Ok   = err == ESP_OK;
    s.snap.spiBusOk = Lsm6dsv16x::acquireBus();

    for (;;) {
        const uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kIntTimeoutMs));
        if (notified > 0) {
            ++s.snap.imu.cycles;
        } else {
            ++s.snap.imu.intTimeouts;
        }
        handleRequests(s);
        drainFifo(s);
        s.snap.tUs = esp_timer_get_time();
        gSnap.write(s.snap);
    }
}

bool send(const Request &r)
{
    return gQueue != nullptr && xQueueSend(gQueue, &r, pdMS_TO_TICKS(100)) == pdTRUE;
}

uint32_t nextId()
{
    static uint32_t id = 0;
    return ++id;
}

}  // namespace

bool ControlTask::start(float mountPitch)
{
    gStartMountPitch = mountPitch;
    gQueue           = xQueueCreate(4, sizeof(Request));
    if (gQueue == nullptr) return false;
    return xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                                   configMAX_PRIORITIES - 2, &gTask, 1) == pdPASS;
}

bool ControlTask::read(Snapshot &out) { return gSnap.version() > 0 && gSnap.read(out); }

bool ControlTask::requestGameVector(bool on)
{
    return send({ReqKind::GameVector, on ? 1u : 0u, 0});
}

bool ControlTask::whoAmI(uint32_t n, WhoAmIResult &out)
{
    if (n == 0) return false;
    const Request r = {ReqKind::WhoAmI, n, nextId()};
    if (!send(r)) return false;

    // kWhoAmIPerCycle reads per ~1 ms cycle, plus margin.
    const TickType_t timeout = pdMS_TO_TICKS(1000 + n / kWhoAmIPerCycle * 2);
    const TickType_t t0      = xTaskGetTickCount();
    while (xTaskGetTickCount() - t0 < timeout) {
        if (gWho.read(out) && out.id == r.id) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

bool ControlTask::zero(float &newMountPitch)
{
    const Request r = {ReqKind::Zero, 0, nextId()};
    if (!send(r)) return false;
    const TickType_t t0 = xTaskGetTickCount();
    while (xTaskGetTickCount() - t0 < pdMS_TO_TICKS(500)) {
        Snapshot s;
        if (read(s) && s.zeroAck == r.id) {
            newMountPitch = s.est.mountPitch;
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
}

bool ControlTask::peakStart() { return send({ReqKind::PeakStart, 0, 0}); }
