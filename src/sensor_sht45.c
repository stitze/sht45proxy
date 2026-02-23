#include "sensor_sht45.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "sht45";

#define SHT45_ADDR         0x44
#define SHT45_MEASURE_HIGH 0xFD
#define SHT45_SOFT_RESET   0x94

static uint8_t crc8(uint8_t msb, uint8_t lsb)
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

static esp_err_t sht45_init(sensor_driver_t *base)
{
    sensor_sht45_t *drv = (sensor_sht45_t *)base;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT45_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(drv->bus, &dev_cfg, &drv->dev),
        TAG, "add device failed");

    uint8_t rst = SHT45_SOFT_RESET;
    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(drv->dev, &rst, 1, pdMS_TO_TICKS(100)),
        TAG, "soft reset failed");

    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGI(TAG, "SHT45 ready.");
    return ESP_OK;
}

static esp_err_t sht45_read(sensor_driver_t *base, sensor_data_t *out)
{
    sensor_sht45_t *drv = (sensor_sht45_t *)base;
    uint8_t cmd = SHT45_MEASURE_HIGH;
    uint8_t raw[6];

    ESP_RETURN_ON_ERROR(
        i2c_master_transmit(drv->dev, &cmd, 1, pdMS_TO_TICKS(100)),
        TAG, "transmit failed");

    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_RETURN_ON_ERROR(
        i2c_master_receive(drv->dev, raw, 6, pdMS_TO_TICKS(100)),
        TAG, "receive failed");

    if (crc8(raw[0], raw[1]) != raw[2]) {
        ESP_LOGW(TAG, "CRC mismatch: temperature");
        return ESP_ERR_INVALID_CRC;
    }
    if (crc8(raw[3], raw[4]) != raw[5]) {
        ESP_LOGW(TAG, "CRC mismatch: humidity");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t t_raw  = ((uint16_t)raw[0] << 8) | raw[1];
    uint16_t rh_raw = convert_rh(((uint16_t)raw[3] << 8) | raw[4]);

    out->bytes[0] = t_raw >> 8;
    out->bytes[1] = t_raw & 0xFF;
    out->bytes[2] = crc8(out->bytes[0], out->bytes[1]);
    out->bytes[3] = rh_raw >> 8;
    out->bytes[4] = rh_raw & 0xFF;
    out->bytes[5] = crc8(out->bytes[3], out->bytes[4]);

    return ESP_OK;
}

void sensor_sht45_init_driver(sensor_sht45_t *drv, i2c_master_bus_handle_t bus)
{
    drv->base.init = sht45_init;
    drv->base.read = sht45_read;
    drv->bus       = bus;
    drv->dev       = NULL;
}
