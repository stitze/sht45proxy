#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#include "sensor.h"
#include "controller.h"

#ifdef CONFIG_SENSOR_MOCK
#include "sensor_mock.h"
#else
#include "sensor_sht45.h"
#endif

#include "controller_shtc3.h"

static const char *TAG = "i2c-proxy";

#define MASTER_SDA  GPIO_NUM_17
#define MASTER_SCL  GPIO_NUM_16

#define SHTC3_WAKEUP      0x3517
#define SHTC3_SLEEP       0xB098
#define SHTC3_MEASURE_NCS 0x7CA2
#define SHTC3_MEASURE_LP  0x6458

#define HEATER_RH_CREEP_THRESHOLD  80.0f
#define HEATER_RH_CONDENSATION     95.0f
#define HEATER_MAX_TEMP_C          65.0f
#define HEATER_MAX_DUTY_CYCLE      0.09f
#define HEATER_CREEP_INTERVAL_MS   30000
#define HEATER_COOLDOWN_MS         5000

#define SHT45_HEAT_200MW_1S  0x39
#define SHT45_HEAT_20MW_01S  0x15

typedef struct {
    sensor_data_t data;
    bool          valid;
} cache_t;

static cache_t           cache;
static SemaphoreHandle_t cache_mutex;

typedef struct {
    sensor_driver_t     *sensor;
    controller_driver_t *controller;
} task_args_t;

typedef struct {
    int64_t window_start_us;
    int64_t heater_on_us;
} duty_t;

static duty_t duty;

static bool duty_allows(int64_t duration_us)
{
    int64_t now = esp_timer_get_time();
    int64_t win = 10LL * 60 * 1000000;
    if (now - duty.window_start_us >= win) {
        duty.window_start_us = now;
        duty.heater_on_us    = 0;
    }
    return ((float)(duty.heater_on_us + duration_us) / (float)win)
           <= HEATER_MAX_DUTY_CYCLE;
}

static void duty_record(int64_t us) { duty.heater_on_us += us; }

#ifndef CONFIG_SENSOR_MOCK
static void run_heater(sensor_sht45_t *sht, uint8_t cmd,
                       int64_t dur_us, uint32_t wait_ms)
{
    if (!duty_allows(dur_us)) {
        ESP_LOGW(TAG, "Heater skipped: duty cycle exhausted");
        return;
    }
    uint8_t c = cmd;
    if (i2c_master_transmit(sht->dev, &c, 1, pdMS_TO_TICKS(100)) != ESP_OK) {
        ESP_LOGW(TAG, "Heater command failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    duty_record(dur_us);
    uint8_t discard[6];
    i2c_master_receive(sht->dev, discard, 6, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Heater cycle done (cmd=0x%02X), cooling down...", cmd);
    vTaskDelay(pdMS_TO_TICKS(HEATER_COOLDOWN_MS));
}
#endif

static void measure_task(void *arg)
{
    task_args_t         *args       = (task_args_t *)arg;
    sensor_driver_t     *sensor     = args->sensor;
    controller_driver_t *controller = args->controller;

    sensor_data_t data;
    TickType_t    last_wake    = xTaskGetTickCount();
    int64_t       last_heat_ms = 0;

    for (;;) {
        if (sensor->read(sensor, &data) == ESP_OK) {
            if (xSemaphoreTake(cache_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                cache.data  = data;
                cache.valid = true;
                xSemaphoreGive(cache_mutex);
            }

            controller->push(controller, &data);

#ifndef CONFIG_SENSOR_MOCK
            sensor_sht45_t *sht = (sensor_sht45_t *)sensor;
            float t_c = -45.0f + 175.0f
                        * (float)(((uint16_t)data.bytes[0] << 8) | data.bytes[1])
                        / 65535.0f;
            float rh  = 100.0f
                        * (float)(((uint16_t)data.bytes[3] << 8) | data.bytes[4])
                        / 65535.0f;
            int64_t now_ms = esp_timer_get_time() / 1000;

            if (t_c < HEATER_MAX_TEMP_C) {
                if (rh >= HEATER_RH_CONDENSATION) {
                    ESP_LOGI(TAG, "Condensation (RH=%.1f%%), 200mW/1s heater", rh);
                    run_heater(sht, SHT45_HEAT_200MW_1S, 1000000LL, 1100);
                    last_heat_ms = now_ms;
                } else if (rh >= HEATER_RH_CREEP_THRESHOLD
                           && (now_ms - last_heat_ms) >= HEATER_CREEP_INTERVAL_MS) {
                    ESP_LOGI(TAG, "High RH (%.1f%%), 20mW/0.1s creep pulse", rh);
                    run_heater(sht, SHT45_HEAT_20MW_01S, 100000LL, 110);
                    last_heat_ms = now_ms;
                }
            }
#endif

            ESP_LOGD(TAG, "T_raw=%04X  RH_raw=%04X",
                     ((uint16_t)data.bytes[0] << 8) | data.bytes[1],
                     ((uint16_t)data.bytes[3] << 8) | data.bytes[4]);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
    }
}

static void slave_cmd_task(void *arg)
{
    controller_shtc3_t *drv = (controller_shtc3_t *)arg;
    uint8_t rx[4];

    for (;;) {
        if (i2c_slave_receive(drv->dev, rx, sizeof(rx)) != ESP_OK)
            continue;
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
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SHTC3->SHT45 proxy starting (ESP-IDF %s)...",
             esp_get_idf_version());

#ifdef CONFIG_SENSOR_MOCK
    ESP_LOGW(TAG, "*** MOCK SENSOR ACTIVE ***");
#endif

    cache_mutex = xSemaphoreCreateMutex();
    configASSERT(cache_mutex);
    duty.window_start_us = esp_timer_get_time();

#ifdef CONFIG_SENSOR_MOCK
    static sensor_mock_t mock_drv;
    sensor_mock_init_driver(&mock_drv, 22.5f, 55.0f);
    sensor_driver_t *sensor = &mock_drv.base;
#else
    i2c_master_bus_handle_t master_bus;
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_1,
        .sda_io_num        = MASTER_SDA,
        .scl_io_num        = MASTER_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &master_bus));

    static sensor_sht45_t sht45_drv;
    sensor_sht45_init_driver(&sht45_drv, master_bus);
    sensor_driver_t *sensor = &sht45_drv.base;
#endif
    ESP_ERROR_CHECK(sensor->init(sensor));

    static controller_shtc3_t shtc3_drv;
    controller_shtc3_init_driver(&shtc3_drv);
    controller_driver_t *controller = &shtc3_drv.base;
    ESP_ERROR_CHECK(controller->init(controller));

    sensor_data_t init_data;
    if (sensor->read(sensor, &init_data) == ESP_OK) {
        cache.data  = init_data;
        cache.valid = true;
        controller->push(controller, &init_data);
        ESP_LOGI(TAG, "Initial measurement cached.");
    } else {
        uint8_t crc_zero = 0xFF;
        for (int b = 0; b < 8; b++)
            crc_zero = (crc_zero & 0x80) ? (crc_zero << 1) ^ 0x31 : (crc_zero << 1);
        sensor_data_t fallback = {.bytes = {0, 0, crc_zero, 0, 0, crc_zero}};
        controller->push(controller, &fallback);
        ESP_LOGW(TAG, "Initial measurement failed, fallback set.");
    }

    static task_args_t args;
    args.sensor     = sensor;
    args.controller = controller;

    xTaskCreatePinnedToCore(measure_task,   "Measure",  4096, &args,      5, NULL, 0);
    xTaskCreatePinnedToCore(slave_cmd_task, "SlaveCmd", 2048, &shtc3_drv, 4, NULL, 1);
}
