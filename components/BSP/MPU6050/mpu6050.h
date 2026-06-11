#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

// I2C 引脚配置
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           10
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_PORT             I2C_NUM_0

// MPU6050 I2C 地址 (AD0 接地)
#define MPU6050_ADDR                0x68

// MPU6050 寄存器
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_WHO_AM_I        0x75

// 满量程值 (对应配置 ±2g, ±250°/s)
#define ACCEL_FS_SENSITIVITY        16384.0f   // LSB/g
#define GYRO_FS_SENSITIVITY         131.0f     // LSB/(°/s)

// 踝泵运动检测阈值
#define DORSI_THRESHOLD             30.0f      // 背屈角度阈值 (度)
#define PLANTAR_THRESHOLD           22.0f      // 跖屈角度阈值 (度)

typedef struct {
    float accel_x;   // m/s²
    float accel_y;
    float accel_z;
    float gyro_x;    // °/s
    float gyro_y;
    float gyro_z;
} mpu6050_data_t;

esp_err_t mpu6050_init(void);
esp_err_t mpu6050_read(mpu6050_data_t *data);
