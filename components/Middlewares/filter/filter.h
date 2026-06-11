#pragma once
#include <stdint.h>

typedef struct {
    float *buffer;
    uint8_t size;
    uint8_t index;
    uint8_t count;
    float sum;
} moving_avg_t;

void moving_avg_init(moving_avg_t *f, float *buf, uint8_t size);
float moving_avg_update(moving_avg_t *f, float val);

typedef struct {
    float angle;
    float alpha;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    moving_avg_t ma_ay;
    moving_avg_t ma_az;
    moving_avg_t ma_gx;
    moving_avg_t ma_gy;
    moving_avg_t ma_gz;
} comp_filter_t;

void comp_filter_init(comp_filter_t *f, float alpha,
                      float *ma_ay_buf, float *ma_az_buf,
                      float *ma_gx_buf, float *ma_gy_buf, float *ma_gz_buf,
                      uint8_t ma_size);
float comp_filter_update(comp_filter_t *f, float accel_y, float accel_z,
                         float gyro_x, float gyro_y, float gyro_z, float dt);
