#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

static const char *TAG = "i2c-proxy";

#define SLAVE_SDA   GPIO_NUM_13
#define SLAVE_SCL   GPIO_NUM_14
#define MASTER_SDA  GPIO_NUM_17
#define MASTER_SCL  GPIO_NUM_16

#define SHTC3_ADDR         0x70
#define SHT45_ADDR         0x44

#define SHT45_MEASURE_HIGH 0xFD

#define SHT45_HEAT_200MW_1S   0x39
#define SHT45_HEAT_200MW_01S  0x32
#define SHT45_HEAT_110MW_1S   0x2F
#define SHT45_HEAT_110MW_01S  0x24
#define SHT45_HEAT_20MW_1S    0x1E
#define SHT45_HEAT_20MW_01S   0x15

#define SHT45_SOFT_RESET   0x94

#define SHTC3_WAKEUP       0x3517
#define SHTC3_SLEEP        0xB098
#define SHTC3_MEASURE_NCS  0x7CA2
#define SHTC3_MEASURE_LP   0x6458

// Heater policy thresholds
#define HEATER_RH_CREEP_THRESHOLD   80.0f   // %RH – start periodic creep compensation
#define HEATER_RH_CONDENSATION      95.0f   // %RH – condensation likely, use high power
#define HEATER_MAX_TEMP_C           65.0f   // °C  – disable heater above this
#define HEATER_MAX_DUTY_CYCLE       0.09f   // 9%  – stay safely below 10% limit
#define HEATER_CREEP_INTERVAL_MS    30000   // ms  – how often to heat in creep mode
#define HEATER_COOLDOWN_MS          5000    // ms  – wait after heating before next normal meas.

static i2c_master_bus_handle_t master_bus;
static i2c_master_dev_handle_t sht45_dev;
static i2c_slave_dev_handle_t  slave_dev;

typedef struct {
    uint8_t data[6];
    bool    valid;
} sensor_cache_t;

static sensor_cache_t    cache;
static SemaphoreHandle_t cache_mutex;

// Duty cycle tracking
typedef struct {
    int64_t window_start_us;    // start of 10-minute rolling window
    int64_t heater_on_us;       // accumulated heater-on time in window
} duty_tracker_t;

static duty_tracker_t duty = {0, 0};

static uint8_t calc_crc(uint8_t msb, uint8_t lsb)
{
    uint8_t crc = 0xFF;
    uint8_t d[2] = {msb, lsb};
    for (int i = 0; i < 2; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
    return crc;
}

static uint16_t convert_rh(uint16_t raw45)
{
    float rh = -6.0f + 125.0f * (float)raw45 / 65535.0f;
    if (rh < 0.0f)   rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;
    return (uint16_t)(rh / 100.0f * 65535.0f);
}

static float raw_to_rh(uint16_t raw)
{
    float rh = -6.0f + 125.0f * (float)raw / 65535.0f;
    if (rh < 0.0f)   rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;
    return rh;
}

static float raw_to_temp(uint16_t raw)
{
    return -45.0f + 175.0f * (float)raw / 65535.0f;
}

static esp_err_t fetch_sht45(uint8_t out[6])
{
    uint8_t cmd = SHT45_MEASURE_HIGH;
    uint8_t raw[6];

    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(sht45_dev, &cmd, 1, pdMS_TO_TICKS(100)),
        TAG, "SHT45 transmit failed");

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(
        i2c_master_receive(sht45_dev, raw, 6, pdMS_TO_TICKS(100)),
        TAG, "SHT45 receive failed");

    if (calc_crc(raw[0], raw[1]) != raw[2]) {
        ESP_LOGW(TAG, "CRC mismatch: temperature"); return ESP_ERR_INVALID_CRC;
    }
    if (calc_crc(raw[3], raw[4]) != raw[5]) {
        ESP_LOGW(TAG, "CRC mismatch: humidity");    return ESP_ERR_INVALID_CRC;
    }

    uint16_t t_raw  = ((uint16_t)raw[0] << 8) | raw[1];
    uint16_t rh_raw = convert_rh(((uint16_t)raw[3] << 8) | raw[4]);

    out[0] = t_raw >> 8;  out[1] = t_raw & 0xFF;
    out[2] = calc_crc(out[0], out[1]);
    out[3] = rh_raw >> 8; out[4] = rh_raw & 0xFF;
    out[5] = calc_crc(out[3], out[4]);

    return ESP_OK;
}

// Returns true if the duty cycle budget allows heating for duration_ms.
// Uses a rolling 10-minute window.
static bool duty_cycle_allows(int64_t duration_us)
{
    int64_t now = esp_timer_get_time();
    int64_t window_us = 10LL * 60 * 1000000;

    if (now - duty.window_start_us >= window_us) {
        duty.window_start_us = now;
        duty.heater_on_us    = 0;
    }

    float projected = (float)(duty.heater_on_us + duration_us) / (float)window_us;
    return projected <= HEATER_MAX_DUTY_CYCLE;
}

static void duty_record(int64_t duration_us)
{
    duty.heater_on_us += duration_us;
}

// Send a heater command, wait for the built-in measurement to finish, then
// discard the result (measured at elevated temperature, not representative).
static esp_err_t run_heater(uint8_t heater_cmd, int64_t heater_duration_us, uint32_t wait_ms)
{
    if (!duty_cycle_allows(heater_duration_us)) {
        ESP_LOGW(TAG, "Heater skipped: duty cycle budget exhausted");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd = heater_cmd;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(sht45_dev, &cmd, 1, pdMS_TO_TICKS(100)),
        TAG, "Heater command transmit failed");

    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    duty_record(heater_duration_us);

    // Read and discard the measurement taken during heating
    uint8_t discard[6];
    i2c_master_receive(sht45_dev, discard, 6, pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Heater cycle complete (cmd=0x%02X), cooling down...", heater_cmd);
    vTaskDelay(pdMS_TO_TICKS(HEATER_COOLDOWN_MS));

    return ESP_OK;
}

static void push_to_slave_tx(const uint8_t data[6])
{
    esp_err_t ret = i2c_slave_transmit(slave_dev, data, 6, pdMS_TO_TICKS(5));
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "Slave TX push failed: %s", esp_err_to_name(ret));
}

static void measure_task(void *arg)
{
    uint8_t buf[6];
    TickType_t last_wake          = xTaskGetTickCount();
    int64_t    last_heater_ms     = 0;
    uint32_t   high_rh_streak     = 0;   // consecutive 500ms ticks above threshold

    for (;;) {
        if (fetch_sht45(buf) == ESP_OK) {
            float t_c  = raw_to_temp(((uint16_t)buf[0] << 8) | buf[1]);
            // buf already contains SHTC3-converted RH raw – decode back for logic
            float rh   = raw_to_rh(((uint16_t)buf[3] << 8) | buf[4]);

            // Update cache and slave TX buffer
            if (xSemaphoreTake(cache_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                memcpy(cache.data, buf, 6);
                cache.valid = true;
                xSemaphoreGive(cache_mutex);
            }
            push_to_slave_tx(buf);

            ESP_LOGD(TAG, "T=%.2f C  RH=%.2f%%", t_c, rh);

            // Heater decision
            bool heater_allowed = (t_c < HEATER_MAX_TEMP_C);
            int64_t now_ms = esp_timer_get_time() / 1000;

            if (!heater_allowed) {
                high_rh_streak = 0;
            } else if (rh >= HEATER_RH_CONDENSATION) {
                // Condensation: high power, long pulse
                ESP_LOGI(TAG, "Condensation detected (RH=%.1f%%), running 200mW/1s heater", rh);
                run_heater(SHT45_HEAT_200MW_1S, 1000000LL, 1100);
                last_heater_ms = now_ms;
                high_rh_streak = 0;
            } else if (rh >= HEATER_RH_CREEP_THRESHOLD) {
                high_rh_streak++;
                // Only heat periodically – not on every tick
                if ((now_ms - last_heater_ms) >= HEATER_CREEP_INTERVAL_MS) {
                    ESP_LOGI(TAG, "High RH (%.1f%%), running 20mW/0.1s creep compensation", rh);
                    run_heater(SHT45_HEAT_20MW_01S, 100000LL, 110);
                    last_heater_ms = now_ms;
                }
            } else {
                high_rh_streak = 0;
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
    }
}

static void slave_cmd_task(void *arg)
{
    uint8_t rx[4];

    for (;;) {
        esp_err_t ret = i2c_slave_receive(slave_dev, rx, sizeof(rx));
        if (ret != ESP_OK) continue;

        uint16_t cmd = ((uint16_t)rx[0] << 8) | rx[1];
        switch (cmd) {
            case SHTC3_WAKEUP:
            case SHTC3_SLEEP:
                break;
            case SHTC3_MEASURE_NCS:
            case SHTC3_MEASURE_LP:
                ESP_LOGD(TAG, "Measure command 0x%04X", cmd);
                break;
            default:
                ESP_LOGD(TAG, "Unknown command 0x%04X", cmd);
                break;
        }
    }
}

static esp_err_t init_master(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_1,
        .sda_io_num        = MASTER_SDA,
        .scl_io_num        = MASTER_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &master_bus),
                        TAG, "Master bus init failed");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT45_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(master_bus, &dev_cfg, &sht45_dev),
                        TAG, "SHT45 add device failed");

    uint8_t rst = SHT45_SOFT_RESET;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(sht45_dev, &rst, 1, pdMS_TO_TICKS(100)),
        TAG, "SHT45 not responding");
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGI(TAG, "SHT45 ready.");
    return ESP_OK;
}

static esp_err_t init_slave(void)
{
    i2c_slave_config_t slave_cfg = {
        .addr_bit_len   = I2C_ADDR_BIT_LEN_7,
        .clk_source     = I2C_CLK_SRC_DEFAULT,
        .i2c_port       = I2C_NUM_0,
        .send_buf_depth = 64,
        .scl_io_num     = SLAVE_SCL,
        .sda_io_num     = SLAVE_SDA,
        .slave_addr     = SHTC3_ADDR,
    };
    ESP_RETURN_ON_ERROR(i2c_new_slave_device(&slave_cfg, &slave_dev),
                        TAG, "Slave init failed");

    ESP_LOGI(TAG, "I2C slave ready at 0x%02X.", SHTC3_ADDR);
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "SHTC3->SHT45 proxy starting (ESP-IDF %s)...", esp_get_idf_version());

    cache_mutex = xSemaphoreCreateMutex();
    configASSERT(cache_mutex);

    duty.window_start_us = esp_timer_get_time();

    ESP_ERROR_CHECK(init_master());
    ESP_ERROR_CHECK(init_slave());

    uint8_t init_buf[6];
    if (fetch_sht45(init_buf) == ESP_OK) {
        memcpy(cache.data, init_buf, 6);
        cache.valid = true;
        push_to_slave_tx(init_buf);
        ESP_LOGI(TAG, "Initial measurement cached.");
    } else {
        uint8_t fallback[6] = {0, 0, 0, 0, 0, 0};
        fallback[2] = calc_crc(0, 0);
        fallback[5] = calc_crc(0, 0);
        push_to_slave_tx(fallback);
        ESP_LOGW(TAG, "Initial measurement failed, fallback values set.");
    }

    xTaskCreatePinnedToCore(measure_task,   "Measure",  4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(slave_cmd_task, "SlaveCmd", 2048, NULL, 4, NULL, 1);
}