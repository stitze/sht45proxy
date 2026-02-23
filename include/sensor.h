#pragma once
#include <stdint.h>
#include "esp_err.h"

/**
 * sensor_data_t holds a pre-formatted 6-byte SHTC3-compatible response:
 *   [0] T_MSB  [1] T_LSB  [2] T_CRC
 *   [3] RH_MSB [4] RH_LSB [5] RH_CRC
 * Temperature and humidity are already converted from SHT45 raw values.
 */
typedef struct {
    uint8_t bytes[6];
} sensor_data_t;

typedef struct sensor_driver sensor_driver_t;

struct sensor_driver {
    esp_err_t (*init)(sensor_driver_t *drv);
    esp_err_t (*read)(sensor_driver_t *drv, sensor_data_t *out);
};
