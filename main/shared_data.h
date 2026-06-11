#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ZONE_IDLE = 0,
    ZONE_DORSIFLEX = 1,
    ZONE_PLANTARFLEX = -1
} movement_zone_t;

typedef struct {
    float    angle;
    int8_t   zone;
    uint32_t event_id;
    uint32_t count_dorsi;
    uint32_t count_plantar;
} ble_event_t;

typedef struct {
    float      angle;
    float      gyro_x;
    float      gyro_y;
    float      gyro_z;
    float      accel_x;
    float      accel_y;
    float      accel_z;
    int8_t     zone;
    uint32_t   count_dorsi;
    uint32_t   count_plantar;

    ble_event_t ble_event;

    SemaphoreHandle_t mutex;
    SemaphoreHandle_t ble_sem;
    SemaphoreHandle_t i2c_mutex;
} shared_state_t;

extern shared_state_t g_state;

void shared_state_init(void);
