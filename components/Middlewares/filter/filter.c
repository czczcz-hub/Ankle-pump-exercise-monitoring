#include "filter.h"
#include <math.h>

void moving_avg_init(moving_avg_t *f, float *buf, uint8_t size)
{
    f->buffer = buf;
    f->size = size;
    f->index = 0;
    f->count = 0;
    f->sum = 0.0f;
    for (uint8_t i = 0; i < size; i++) {
        buf[i] = 0.0f;
    }
}

float moving_avg_update(moving_avg_t *f, float val)
{
    if (f->count < f->size) {
        f->count++;
    } else {
        f->sum -= f->buffer[f->index];
    }
    f->buffer[f->index] = val;
    f->sum += val;
    f->index = (f->index + 1) % f->size;
    return f->sum / (float)f->count;
}

void comp_filter_init(comp_filter_t *f, float alpha,
                      float *ma_ay_buf, float *ma_az_buf,
                      float *ma_gx_buf, float *ma_gy_buf, float *ma_gz_buf,
                      uint8_t ma_size)
{
    f->angle = 0.0f;
    f->alpha = alpha;
    f->gyro_x = 0.0f;
    f->gyro_y = 0.0f;
    f->gyro_z = 0.0f;
    moving_avg_init(&f->ma_ay, ma_ay_buf, ma_size);
    moving_avg_init(&f->ma_az, ma_az_buf, ma_size);
    moving_avg_init(&f->ma_gx, ma_gx_buf, ma_size);
    moving_avg_init(&f->ma_gy, ma_gy_buf, ma_size);
    moving_avg_init(&f->ma_gz, ma_gz_buf, ma_size);
}

float comp_filter_update(comp_filter_t *f, float accel_y, float accel_z,
                         float gyro_x, float gyro_y, float gyro_z, float dt)
{
    float ay = moving_avg_update(&f->ma_ay, accel_y);
    float az = moving_avg_update(&f->ma_az, accel_z);
    float angle_acc = atan2f(ay, az) * 180.0f / (float)M_PI;

    f->gyro_x = moving_avg_update(&f->ma_gx, gyro_x);
    f->gyro_y = moving_avg_update(&f->ma_gy, gyro_y);
    f->gyro_z = moving_avg_update(&f->ma_gz, gyro_z);

    f->angle = f->alpha * (f->angle + f->gyro_x * dt) + (1.0f - f->alpha) * angle_acc;
    return f->angle;
}
