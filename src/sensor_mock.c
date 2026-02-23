#include "sensor_mock.h"
#include "esp_log.h"

static const char *TAG = "sensor_mock";

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

static esp_err_t mock_init(sensor_driver_t *base)
{
    sensor_mock_t *drv = (sensor_mock_t *)base;
    ESP_LOGI(TAG, "Mock sensor ready (T=%.1f C, RH=%.1f %%)", drv->temp_c, drv->rh_pct);
    return ESP_OK;
}

static esp_err_t mock_read(sensor_driver_t *base, sensor_data_t *out)
{
    sensor_mock_t *drv = (sensor_mock_t *)base;

    /* Encode using SHTC3 formulae (what the controller expects):
     *   T_raw  = (T + 45) / 175 * 65535
     *   RH_raw = RH / 100 * 65535
     */
    float t = drv->temp_c;
    float rh = drv->rh_pct;
    if (rh < 0.0f)   rh = 0.0f;
    if (rh > 100.0f) rh = 100.0f;

    uint16_t t_raw  = (uint16_t)((t + 45.0f) / 175.0f * 65535.0f);
    uint16_t rh_raw = (uint16_t)(rh / 100.0f * 65535.0f);

    out->bytes[0] = t_raw >> 8;
    out->bytes[1] = t_raw & 0xFF;
    out->bytes[2] = crc8(out->bytes[0], out->bytes[1]);
    out->bytes[3] = rh_raw >> 8;
    out->bytes[4] = rh_raw & 0xFF;
    out->bytes[5] = crc8(out->bytes[3], out->bytes[4]);

    ESP_LOGD(TAG, "Mock read: T=%.1f C  RH=%.1f %%", t, rh);
    return ESP_OK;
}

void sensor_mock_init_driver(sensor_mock_t *drv, float temp_c, float rh_pct)
{
    drv->base.init = mock_init;
    drv->base.read = mock_read;
    drv->temp_c    = temp_c;
    drv->rh_pct    = rh_pct;
}
