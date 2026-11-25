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
#include "imu/FifoDecoder.h"
#include "imu/Lsm6dsv16x.h"
#include "util/Half.h"
#include "util/Snapshot.h"

namespace {

constexpr uint32_t kIntTimeoutMs    = 3;   // about 3 IMU periods
constexpr uint32_t kWhoAmIPerCycle  = 8;
constexpr int kMaxDrainPasses       = 4;
constexpr float kDegToRad           = 0.017453292519943f;

enum class ReqKind : uint8_t { GameVector, WhoAmI };
struct Request {
    ReqKind kind;
    uint32_t arg;
    uint32_t id;
};

TaskHandle_t gTask    = nullptr;
QueueHandle_t gQueue  = nullptr;
::Snapshot<ControlTask::Snapshot> gSnap;
::Snapshot<ControlTask::WhoAmIResult> gWho;
volatile int64_t gIntUs = 0;  // last INT1 time (Stage 5 latency)

void IRAM_ATTR int1Isr(void *)
{
    gIntUs              = esp_timer_get_time();
    BaseType_t woken    = pdFALSE;
    vTaskNotifyGiveFromISR(gTask, &woken);
    portYIELD_FROM_ISR(woken);
}

// State owned by the control task.
struct State {
    ControlTask::Snapshot snap{};
    FifoDecoder decoder;
    uint32_t pairsThisDrain = 0;
    bool overrunLatched     = false;

    // WHO_AM_I request in progress.
    uint32_t whoRemaining = 0;
    ControlTask::WhoAmIResult who{};

    // FifoDecoder sink.
    void onPair(const ImuPair &p)
    {
        float g[3], a[3];
        for (int i = 0; i < 3; ++i) {
            g[i] = p.gyro[i] * (Lsm6dsv16x::kGyroDpsPerLsb * kDegToRad);
            a[i] = p.accel[i] * Lsm6dsv16x::kAccelGPerLsb;
        }
        applyAxisMap(kRsb, g, snap.gyro);
        applyAxisMap(kRsb, a, snap.accel);
        snap.imuTs = p.ts;
        ++snap.imu.pairs;
        ++pairsThisDrain;
    }
    void onGravity(const int16_t v[3])
    {
        float g[3];
        for (int i = 0; i < 3; ++i) g[i] = v[i] * (Lsm6dsv16x::kSflpGravMgPerLsb * 0.001f);
        applyAxisMap(kRsb, g, snap.gravity);
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

}  // namespace

bool ControlTask::start()
{
    gQueue = xQueueCreate(4, sizeof(Request));
    if (gQueue == nullptr) return false;
    return xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                                   configMAX_PRIORITIES - 2, &gTask, 1) == pdPASS;
}

bool ControlTask::read(Snapshot &out) { return gSnap.version() > 0 && gSnap.read(out); }

bool ControlTask::requestGameVector(bool on)
{
    if (gQueue == nullptr) return false;
    const Request r = {ReqKind::GameVector, on ? 1u : 0u, 0};
    return xQueueSend(gQueue, &r, pdMS_TO_TICKS(100)) == pdTRUE;
}

bool ControlTask::whoAmI(uint32_t n, WhoAmIResult &out)
{
    static uint32_t nextId = 1;
    if (gQueue == nullptr || n == 0) return false;
    const Request r = {ReqKind::WhoAmI, n, nextId++};
    if (xQueueSend(gQueue, &r, pdMS_TO_TICKS(100)) != pdTRUE) return false;

    // kWhoAmIPerCycle reads per ~1 ms cycle, plus margin.
    const TickType_t timeout = pdMS_TO_TICKS(1000 + n / kWhoAmIPerCycle * 2);
    const TickType_t t0      = xTaskGetTickCount();
    while (xTaskGetTickCount() - t0 < timeout) {
        if (gWho.read(out) && out.id == r.id) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}
