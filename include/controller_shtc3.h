#pragma once
#include "controller.h"
#include "driver/i2c_slave.h"

typedef struct {
    controller_driver_t    base;
    i2c_slave_dev_handle_t dev;
} controller_shtc3_t;

void controller_shtc3_init_driver(controller_shtc3_t *drv);
