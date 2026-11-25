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

const stmdev_ctx_t &Lsm6dsv16x::ctx() { return gCtx; }
