#include "imu/Lsm6dsv16x.h"

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <string.h>

#include "PinConfig.h"
#include "util/RunOnCore.h"

namespace {

constexpr spi_host_device_t kHost = SPI2_HOST;
// No DMA: transfers are register reads and, from Stage 2, FIFO bursts of a
// few words; the SPI FIFO holds 64 bytes (SOC_SPI_MAXIMUM_BUFFER_SIZE).
constexpr int kMaxTransfer = 64;
static_assert(Lsm6dsv16x::kMaxWordsPerRead * 7 <= kMaxTransfer, "FIFO burst exceeds SPI buffer");

spi_device_handle_t gDev = nullptr;
stmdev_ctx_t gCtx    = {};

int32_t spiWrite(void *, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    if (gDev == nullptr || len > kMaxTransfer) return -1;
    spi_transaction_t t = {};
    t.addr      = reg & 0x7F;
    t.length    = static_cast<size_t>(len) * 8;
    t.tx_buffer = buf;
    return spi_device_polling_transmit(gDev, &t) == ESP_OK ? 0 : -1;
}

int32_t spiRead(void *, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (gDev == nullptr || len > kMaxTransfer) return -1;
    spi_transaction_t t = {};
    t.addr      = reg | 0x80;  // read bit (spec §5.1)
    t.length    = static_cast<size_t>(len) * 8;
    t.rx_buffer = buf;
    return spi_device_polling_transmit(gDev, &t) == ESP_OK ? 0 : -1;
}

void delayMs(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

struct InitResult {
    esp_err_t err;
    const char *step;
};

void initOnCore1(void *p)
{
    InitResult &r = *static_cast<InitResult *>(p);

    // CS high before any other SPI traffic (spec §5.1): a low CS at power-up
    // would leave the IMU deciding between its I2C and SPI interfaces.
    gpio_set_level(PIN_IMU_CS, 1);
    gpio_set_direction(PIN_IMU_CS, GPIO_MODE_OUTPUT);

    spi_bus_config_t bus = {};
    bus.mosi_io_num     = PIN_IMU_MOSI;
    bus.miso_io_num     = PIN_IMU_MISO;
    bus.sclk_io_num     = PIN_IMU_SCK;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = kMaxTransfer;
    r.err  = spi_bus_initialize(kHost, &bus, SPI_DMA_DISABLED);
    r.step = "spi_bus_initialize";
    if (r.err != ESP_OK) return;

    spi_device_interface_config_t dev = {};
    dev.address_bits   = 8;  // register address (+ read bit) as the address phase
    dev.mode           = 3;  // CPOL = CPHA = 1; the LSM6DSV16X also takes mode 0
    dev.clock_speed_hz = static_cast<int>(Lsm6dsv16x::kSpiClockHz);
    dev.spics_io_num   = PIN_IMU_CS;
    dev.queue_size     = 1;
    r.err  = spi_bus_add_device(kHost, &dev, &gDev);
    r.step = "spi_bus_add_device";
    if (r.err != ESP_OK) {
        gDev = nullptr;
        spi_bus_free(kHost);
    }
}

}  // namespace

bool Lsm6dsv16x::begin()
{
    InitResult r = {ESP_FAIL, "task create"};
    RunOnCore::run(1, initOnCore1, &r, "imu init");
    if (r.err != ESP_OK) {
        Serial.printf("IMU: %s failed: %s\n", r.step, esp_err_to_name(r.err));
        return false;
    }

    gCtx.write_reg = spiWrite;
    gCtx.read_reg  = spiRead;
    gCtx.mdelay    = delayMs;
    gCtx.handle    = nullptr;

    int actualKhz = 0;
    spi_device_get_actual_freq(gDev, &actualKhz);
    Serial.printf("IMU: SPI2 mode 3, %d kHz, CS %d SCK %d MOSI %d MISO %d\n", actualKhz,
                  PIN_IMU_CS, PIN_IMU_SCK, PIN_IMU_MOSI, PIN_IMU_MISO);
    return true;
}

bool Lsm6dsv16x::readWhoAmI(uint8_t &id)
{
    id = 0;
    return lsm6dsv16x_get_device_id(&gCtx, &id) == 0;
}

namespace {
int8_t gFreqFine = 0;

// Logs and returns false if an ST driver call failed.
bool step(int32_t ret, const char *what)
{
    if (ret != 0) Serial.printf("IMU: %s failed (%ld)\n", what, static_cast<long>(ret));
    return ret == 0;
}
}  // namespace

bool Lsm6dsv16x::configure()
{
    const stmdev_ctx_t *c = &gCtx;
    bool ok = true;

    // 1. Software reset, then wait for it to complete (the driver polls).
    ok &= step(lsm6dsv16x_sw_reset(c), "software reset");
    vTaskDelay(pdMS_TO_TICKS(2));

    // Interface: push-pull, active-high interrupt pins; I2C/I3C off (SPI only).
    lsm6dsv16x_if_cfg_t ifCfg = {};
    ok &= step(lsm6dsv16x_read_reg(c, LSM6DSV16X_IF_CFG, reinterpret_cast<uint8_t *>(&ifCfg), 1),
               "read IF_CFG");
    ifCfg.pp_od           = 0;
    ifCfg.h_lactive       = 0;
    ifCfg.i2c_i3c_disable = 1;
    ok &= step(lsm6dsv16x_write_reg(c, LSM6DSV16X_IF_CFG, reinterpret_cast<uint8_t *>(&ifCfg), 1),
               "write IF_CFG");

    // 2. BDU on, auto-increment on.
    ok &= step(lsm6dsv16x_set_block_data_update(c, PROPERTY_ENABLE), "BDU");
    ok &= step(lsm6dsv16x_set_auto_increment(c, PROPERTY_ENABLE), "auto-increment");

    // 3, 4. Full scales.
    ok &= step(lsm6dsv16x_set_xl_full_scale(c, LSM6DSV16X_8g), "accel +-8 g");
    ok &= step(lsm6dsv16x_set_gy_full_scale(c, LSM6DSV16X_2000dps), "gyro +-2000 dps");

    // 6. SFLP at 480 Hz, gravity vector only in the FIFO.
    ok &= step(lsm6dsv16x_set_sflp_data_rate(c, LSM6DSV16X_SFLP_480Hz), "SFLP 480 Hz");
    ok &= step(lsm6dsv16x_set_sflp_game_rotation(c, PROPERTY_ENABLE), "SFLP enable");
    lsm6dsv16x_fifo_sflp_raw_t sflp = {};
    sflp.gravity = 1;
    ok &= step(lsm6dsv16x_set_fifo_sflp_batch(c, sflp), "SFLP gravity batch");

    // 7, 8. FIFO: accel and gyro at 960 Hz, timestamps, watermark, INT1.
    ok &= step(lsm6dsv16x_set_fifo_xl_batch(c, LSM6DSV16X_XL_BATCHED_AT_960Hz), "FIFO accel");
    ok &= step(lsm6dsv16x_set_fifo_gy_batch(c, LSM6DSV16X_GY_BATCHED_AT_960Hz), "FIFO gyro");
    ok &= step(lsm6dsv16x_set_timestamp(c, PROPERTY_ENABLE), "timestamp");
    ok &= step(lsm6dsv16x_set_fifo_timestamp_batch(c, LSM6DSV16X_TMSTMP_DEC_1), "FIFO timestamp");
    ok &= step(lsm6dsv16x_set_fifo_watermark(c, kFifoWatermarkWords), "FIFO watermark");
    lsm6dsv16x_pin_int_route_t int1 = {};
    int1.fifo_th = 1;
    ok &= step(lsm6dsv16x_set_pin_int1_route(c, &int1), "INT1 route");
    ok &= step(lsm6dsv16x_set_fifo_mode(c, LSM6DSV16X_STREAM_MODE), "FIFO continuous");

    // 5. Accel and gyro 960 Hz, high-performance. Last, so data starts with
    // everything else in place.
    ok &= step(lsm6dsv16x_set_xl_mode(c, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD), "accel HP mode");
    ok &= step(lsm6dsv16x_set_gy_mode(c, LSM6DSV16X_GY_HIGH_PERFORMANCE_MD), "gyro HP mode");
    ok &= step(lsm6dsv16x_set_xl_data_rate(c, LSM6DSV16X_ODR_AT_960Hz), "accel 960 Hz");
    ok &= step(lsm6dsv16x_set_gy_data_rate(c, LSM6DSV16X_ODR_AT_960Hz), "gyro 960 Hz");

    ok &= step(lsm6dsv16x_get_odr_cal_reg(c, &gFreqFine), "read INTERNAL_FREQ_FINE");

    Serial.printf("IMU: %s: +-8 g, +-2000 dps, 960 Hz HP, SFLP gravity 480 Hz, FIFO wtm %u, "
                  "freq fine %d (ODR %.2f Hz)\n",
                  ok ? "configured" : "configuration FAILED", kFifoWatermarkWords, gFreqFine,
                  kOdrHz * (1.0f + 0.0013f * gFreqFine));
    return ok;
}

int8_t Lsm6dsv16x::freqFine() { return gFreqFine; }

bool Lsm6dsv16x::acquireBus()
{
    return gDev != nullptr && spi_device_acquire_bus(gDev, portMAX_DELAY) == ESP_OK;
}

bool Lsm6dsv16x::readFifoStatus(lsm6dsv16x_fifo_status_t &status)
{
    return lsm6dsv16x_get_fifo_status(&gCtx, &status) == 0;
}

bool Lsm6dsv16x::readFifoWords(uint8_t *buf, uint16_t words)
{
    if (words == 0 || words > kMaxWordsPerRead) return false;
    // FIFO_DATA_OUT_TAG..Z_H is 7 registers; with auto-increment the address
    // wraps from 0x7E back to 0x78, so one burst reads several words.
    return spiRead(nullptr, LSM6DSV16X_FIFO_DATA_OUT_TAG, buf, words * 7) == 0;
}

bool Lsm6dsv16x::setGameVectorBatch(bool on)
{
    lsm6dsv16x_fifo_sflp_raw_t sflp = {};
    sflp.gravity       = 1;
    sflp.game_rotation = on ? 1 : 0;
    return lsm6dsv16x_set_fifo_sflp_batch(&gCtx, sflp) == 0;
}

const stmdev_ctx_t &Lsm6dsv16x::ctx() { return gCtx; }
