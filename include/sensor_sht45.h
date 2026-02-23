#pragma once
#include "sensor.h"
#include "driver/i2c_master.h"

typedef struct {
    sensor_driver_t base;           /* must be first */
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} sensor_sht45_t;

void sensor_sht45_init_driver(sensor_sht45_t *drv, i2c_master_bus_handle_t bus);
