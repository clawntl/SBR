#include "can/CanBus.h"

#include <ACAN_ESP32FD.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include "PinConfig.h"
#include "util/RunOnCore.h"

namespace {

using Mode = ACAN_ESP32FD_Settings::ModuleMode;

constexpr uint32_t kFilterId   = 0x100;  // driver telemetry 0x100-0x10F (spec §4.1)
constexpr uint32_t kFilterMask = 0x7F0;

constexpr uint8_t kTagPing     = 0x01;
constexpr uint8_t kTagEcho     = 0x02;
constexpr uint8_t kTagLoop     = 0x03;

constexpr int kServicePeriodMs  = 10;
constexpr int kLoopFrames       = 64;
constexpr int kLoopWaitMs       = 20;

ACAN_ESP32FD can0;

// Guards can0 between the service task and console commands (canloop stops
// and restarts the controller). Stage 1 only: from Stage 4 the control task
// owns the bus and takes no locks (spec §8.2 rule 3).
SemaphoreHandle_t gLock = nullptr;

bool gUp            = false;
volatile bool gTest = false;
volatile bool gTestJustEnabled = false;

struct Timing {
    uint32_t beginErr     = 0;
    uint32_t consistency  = 0;
    uint32_t bitRate      = 0;
    float samplePct       = 0;
    bool filterOk         = false;
    Mode mode             = ACAN_ESP32FD_Settings::NORMAL;
} gTiming;

struct Counters {
    uint32_t rx          = 0;
    uint32_t pings       = 0;
    uint32_t pingFail    = 0;
    uint32_t echoes      = 0;
    uint32_t echoFail    = 0;
    uint32_t busOffs     = 0;
} gCount;

uint32_t gPingSeq = 0;

const char *modeName(Mode m)
{
    switch (m) {
        case ACAN_ESP32FD_Settings::NORMAL:           return "normal";
        case ACAN_ESP32FD_Settings::LOOP_BACK_NO_ACK: return "loopback (no ACK)";
        case ACAN_ESP32FD_Settings::LOOP_BACK:        return "loopback";
        case ACAN_ESP32FD_Settings::LISTEN_ONLY:      return "listen only";
    }
    return "?";
}

struct BeginJob {
    const ACAN_ESP32FD_Settings *settings;
    uint32_t err;
};

void beginOnCore1(void *p)
{
    BeginJob &job = *static_cast<BeginJob *>(p);
    job.err       = can0.begin(*job.settings);
}

// Call with gLock held and can0 stopped.
bool startLocked(Mode mode)
{
    ACAN_ESP32FD_Settings s(CanBus::kBitRate, CanBus::kSamplePermill);
    s.mTxPin      = PIN_CAN_TX;
    s.mRxPin      = PIN_CAN_RX;
    s.mModuleMode = mode;
    gTiming.filterOk = s.mFilters.addMaskFilter(kFilterId, kFilterMask, false);

    BeginJob job = {&s, 0xFFFFFFFFu};
    RunOnCore::run(1, beginOnCore1, &job, "can begin");

    gTiming.beginErr    = job.err;
    gTiming.consistency = s.checkBitSettingConsistency();
    gTiming.bitRate     = s.actualArbitrationBitRate();
    gTiming.samplePct   = s.arbitrationSamplePointFromBitStart();
    gTiming.mode        = mode;
    gUp                 = (job.err == 0) && gTiming.filterOk;
    return gUp;
}

void printBeginLine()
{
    Serial.printf("CAN: begin %s (err 0x%08lX, settings 0x%08lX), %s, %lu bit/s, sample point "
                  "%.1f %%, filter 0x%03lX/0x%03lX %s\n",
                  gTiming.beginErr == 0 ? "ok" : "FAILED",
                  static_cast<unsigned long>(gTiming.beginErr),
                  static_cast<unsigned long>(gTiming.consistency), modeName(gTiming.mode),
                  static_cast<unsigned long>(gTiming.bitRate), gTiming.samplePct,
                  static_cast<unsigned long>(kFilterId), static_cast<unsigned long>(kFilterMask),
                  gTiming.filterOk ? "ok" : "REJECTED");
}

void putU32Be(uint8_t *p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

// gLock held.
bool sendPingLocked()
{
    CANMessage m;
    m.id      = CanBus::kTestId;
    m.len     = 5;
    m.data[0] = kTagPing;
    putU32Be(&m.data[1], gPingSeq);
    if (gUp && can0.tryToSend(m)) {
        ++gPingSeq;
        ++gCount.pings;
        return true;
    }
    ++gCount.pingFail;
    return false;
}

// gLock held.
void echoLocked(const CANMessage &rx)
{
    CANMessage m;
    m.id      = CanBus::kTestId;
    m.data[0] = kTagEcho;
    m.data[1] = static_cast<uint8_t>(rx.id - kFilterId);
    const uint8_t n = rx.len < 6 ? rx.len : 6;
    memcpy(&m.data[2], rx.data, n);
    m.len = static_cast<uint8_t>(2 + n);
    if (can0.tryToSend(m)) {
        ++gCount.echoes;
    } else {
        ++gCount.echoFail;
    }
}

// Core 0. Drains RX, sends the periodic ping, handles bus-off. Stage 1
// scaffolding: the supervisor and control tasks take this over in Stage 4.
void serviceTask(void *)
{
    TickType_t wake      = xTaskGetTickCount();
    TickType_t lastPing  = wake;
    bool wasBusOff       = false;
    uint32_t offTicks    = 0;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(kServicePeriodMs));
        xSemaphoreTake(gLock, portMAX_DELAY);
        if (gUp) {
            CANMessage m;
            while (can0.receive(m)) {
                ++gCount.rx;
                if (gTest) echoLocked(m);
            }

            if (gTest) {
                const TickType_t now = xTaskGetTickCount();
                if (gTestJustEnabled) {
                    gTestJustEnabled = false;
                    lastPing         = now;
                    sendPingLocked();
                } else if (now - lastPing >= pdMS_TO_TICKS(CanBus::kPingIntervalMs)) {
                    lastPing += pdMS_TO_TICKS(CanBus::kPingIntervalMs);
                    if (now - lastPing >= pdMS_TO_TICKS(CanBus::kPingIntervalMs)) lastPing = now;
                    sendPingLocked();
                }
            }

            const bool off = can0.isBusOff();
            if (off) {
                if (!wasBusOff) {
                    ++gCount.busOffs;
                    Serial.println("CAN: bus-off, requesting recovery");
                    can0.recoverFromBusOff();
                    offTicks = 0;
                } else if (++offTicks % 100 == 0) {
                    can0.recoverFromBusOff();  // retry every second while off
                }
            } else if (wasBusOff) {
                Serial.println("CAN: recovered from bus-off");
            }
            wasBusOff = off;
        }
        xSemaphoreGive(gLock);
    }
}

void fillLoopFrame(CANMessage &m, int i)
{
    m.id  = kFilterId + (i & 0x0F);
    m.len = static_cast<uint8_t>(i % 9);
    for (int k = 0; k < 8; ++k) m.data[k] = static_cast<uint8_t>((i * 31 + k * 7) ^ 0xA5);
}

// Waits up to kLoopWaitMs for one frame. gLock held.
bool waitFrame(CANMessage &m)
{
    for (int t = 0; t < kLoopWaitMs; ++t) {
        if (can0.receive(m)) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return can0.receive(m);
}

}  // namespace

bool CanBus::begin()
{
    if (gLock == nullptr) gLock = xSemaphoreCreateMutex();
    xSemaphoreTake(gLock, portMAX_DELAY);
    const bool ok = startLocked(ACAN_ESP32FD_Settings::NORMAL);
    xSemaphoreGive(gLock);
    printBeginLine();
    if (!ok) Serial.println("CAN: not running; motor use is blocked");

    static bool taskStarted = false;
    if (!taskStarted) {
        taskStarted = true;
        xTaskCreatePinnedToCore(serviceTask, "can svc", 4096, nullptr, 4, nullptr, 0);
    }
    return ok;
}

bool CanBus::isUp() { return gUp; }

bool CanBus::ping()
{
    xSemaphoreTake(gLock, portMAX_DELAY);
    const bool ok = sendPingLocked();
    xSemaphoreGive(gLock);
    return ok;
}

void CanBus::setTest(bool on)
{
    if (on && !gTest) gTestJustEnabled = true;
    gTest = on;
}

bool CanBus::testOn() { return gTest; }

bool CanBus::loopbackSelfTest()
{
    xSemaphoreTake(gLock, portMAX_DELAY);

    can0.end();
    gUp = false;
    if (!startLocked(ACAN_ESP32FD_Settings::LOOP_BACK_NO_ACK)) {
        xSemaphoreGive(gLock);
        printBeginLine();
        Serial.println("canloop: FAIL (could not start in loopback)");
        CanBus::begin();
        return false;
    }

    int sendFail = 0, back = 0, wrong = 0, missing = 0;
    CANMessage rx;
    while (can0.receive(rx)) {}  // nothing should be waiting, but start clean

    for (int i = 0; i < kLoopFrames; ++i) {
        CANMessage tx;
        fillLoopFrame(tx, i);
        if (!can0.tryToSend(tx)) {
            ++sendFail;
            continue;
        }
        if (!waitFrame(rx)) {
            ++missing;
            continue;
        }
        const bool same = rx.id == tx.id && !rx.ext && !rx.rtr && rx.len == tx.len &&
                          memcmp(rx.data, tx.data, tx.len) == 0;
        if (same) {
            ++back;
        } else {
            ++wrong;
        }
    }

    // Filter probe: 0x7F0 is outside 0x100-0x10F, so it must not come back.
    CANMessage probe;
    probe.id      = kTestId;
    probe.len     = 2;
    probe.data[0] = kTagLoop;
    probe.data[1] = 0xFF;
    const bool probeSent  = can0.tryToSend(probe);
    const bool probeLeaked = probeSent && waitFrame(rx);

    const uint32_t busErrors = can0.busErrorCount();
    const bool overflow      = can0.driverReceiveFIFODidOverflow();

    can0.end();
    gUp = false;
    startLocked(ACAN_ESP32FD_Settings::NORMAL);
    xSemaphoreGive(gLock);

    const bool pass = back == kLoopFrames && probeSent && !probeLeaked && busErrors == 0 &&
                      !overflow;
    Serial.printf("canloop: %d/%d frames back intact (%d wrong, %d missing, %d not sent)\n", back,
                  kLoopFrames, wrong, missing, sendFail);
    Serial.printf("canloop: filter %s 0x%03lX%s, bus errors %lu, rx overflow %s\n",
                  probeLeaked ? "LET THROUGH" : "rejected", static_cast<unsigned long>(kTestId),
                  probeSent ? "" : " (probe not sent)", static_cast<unsigned long>(busErrors),
                  overflow ? "yes" : "no");
    printBeginLine();
    Serial.printf("canloop: %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

void CanBus::printStatus()
{
    xSemaphoreTake(gLock, portMAX_DELAY);
    const bool up          = gUp;
    const uint32_t busErr  = can0.busErrorCount();
    const uint16_t tec     = can0.txErrorCounter();
    const uint16_t rec     = can0.rxErrorCounter();
    const bool off         = can0.isBusOff();
    const uint32_t rxPeak  = can0.driverReceiveFIFOPeakCount();
    const uint32_t rxSize  = can0.driverReceiveFIFOSize();
    const bool rxOverflow  = can0.driverReceiveFIFODidOverflow();
    const uint32_t txPeak  = can0.driverTransmitFIFOPeakCount();
    const Counters c       = gCount;
    xSemaphoreGive(gLock);

    Serial.printf("CAN0: %s, %s, %lu bit/s, sample point %.1f %%\n", up ? "up" : "DOWN",
                  modeName(gTiming.mode), static_cast<unsigned long>(gTiming.bitRate),
                  gTiming.samplePct);
    Serial.printf("  bus errors %lu, TEC %u, REC %u, bus-off %s (%lu events)\n",
                  static_cast<unsigned long>(busErr), tec, rec, off ? "YES" : "no",
                  static_cast<unsigned long>(c.busOffs));
    Serial.printf("  rx %lu (FIFO peak %lu/%lu, overflow %s), tx FIFO peak %lu\n",
                  static_cast<unsigned long>(c.rx), static_cast<unsigned long>(rxPeak),
                  static_cast<unsigned long>(rxSize), rxOverflow ? "YES" : "no",
                  static_cast<unsigned long>(txPeak));
    Serial.printf("  test %s: pings %lu (%lu failed), echoes %lu (%lu failed)\n",
                  gTest ? "on" : "off", static_cast<unsigned long>(c.pings),
                  static_cast<unsigned long>(c.pingFail), static_cast<unsigned long>(c.echoes),
                  static_cast<unsigned long>(c.echoFail));
}
