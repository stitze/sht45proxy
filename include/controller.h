#pragma once
#include "esp_err.h"
#include "sensor.h"

typedef struct controller_driver controller_driver_t;

struct controller_driver {
    esp_err_t (*init)(controller_driver_t *drv);
    esp_err_t (*push)(controller_driver_t *drv, const sensor_data_t *data);
};
