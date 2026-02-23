#include "controller_shtc3.h"
#include "esp_log.h"
#include "esp_check.h"
#include <freertos/FreeRTOS.h> 
#include <freertos/task.h>

static const char *TAG = "ctrl_shtc3";

#define SLAVE_SDA   GPIO_NUM_21
#define SLAVE_SCL   GPIO_NUM_22
#define SHTC3_ADDR  0x70

static esp_err_t shtc3_init(controller_driver_t *base)
{
    controller_shtc3_t *drv = (controller_shtc3_t *)base;

    i2c_slave_config_t cfg = {
        .addr_bit_len   = I2C_ADDR_BIT_LEN_7,
        .clk_source     = I2C_CLK_SRC_DEFAULT,
        .i2c_port       = I2C_NUM_0,
        .send_buf_depth = 64,
        .scl_io_num     = SLAVE_SCL,
        .sda_io_num     = SLAVE_SDA,
        .slave_addr     = SHTC3_ADDR,
    };
    ESP_RETURN_ON_ERROR(i2c_new_slave_device(&cfg, &drv->dev),
                        TAG, "Slave init failed");

    ESP_LOGI(TAG, "I2C slave ready at 0x%02X.", SHTC3_ADDR);
    return ESP_OK;
}

static esp_err_t shtc3_push(controller_driver_t *base, const sensor_data_t *data)
{
    controller_shtc3_t *drv = (controller_shtc3_t *)base;
    esp_err_t ret = i2c_slave_transmit(drv->dev, data->bytes, 6, pdMS_TO_TICKS(5));
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "TX failed: %s", esp_err_to_name(ret));
    return ret;
}

void controller_shtc3_init_driver(controller_shtc3_t *drv)
{
    drv->base.init = shtc3_init;
    drv->base.push = shtc3_push;
    drv->dev       = NULL;
}
