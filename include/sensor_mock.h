#pragma once
#include "sensor.h"

typedef struct {
    sensor_driver_t base;
    float temp_c;
    float rh_pct;
} sensor_mock_t;

/**
 * Initialise the mock driver with fixed return values.
 * temp_c and rh_pct are returned on every read().
 * Change drv->temp_c / drv->rh_pct at runtime to simulate conditions.
 */
void sensor_mock_init_driver(sensor_mock_t *drv, float temp_c, float rh_pct);
