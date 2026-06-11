#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "mpu6050";

esp_err_t mpu6050_init(void)
{
    // 初始化 I2C 主机
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0));

    // 检查 WHO_AM_I
    uint8_t who_am_i;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_REG_WHO_AM_I, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &who_am_i, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK || who_am_i != 0x68) {
        ESP_LOGE(TAG, "MPU6050 not found (WHO_AM_I=0x%02x, err=%d)", who_am_i, ret);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MPU6050 found (WHO_AM_I=0x%02x)", who_am_i);

    // 唤醒 MPU6050 (退出睡眠模式)
    uint8_t data[2];
    data[0] = MPU6050_REG_PWR_MGMT_1;
    data[1] = 0x00;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 2, true);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);

    // 配置加速度计量程 ±2g
    data[0] = MPU6050_REG_ACCEL_CONFIG;
    data[1] = 0x00;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 2, true);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);

    // 配置陀螺仪量程 ±250°/s
    data[0] = MPU6050_REG_GYRO_CONFIG;
    data[1] = 0x00;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, 2, true);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000)));
    i2c_cmd_link_delete(cmd);

    ESP_LOGI(TAG, "MPU6050 initialized");
    return ESP_OK;
}

esp_err_t mpu6050_read(mpu6050_data_t *data)
{
    // 从 ACCEL_XOUT_H 开始连续读取 14 字节 (3×Accel + Temp + 3×Gyro)
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buf[14];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 14, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    // 解析原始值 (大端转小端)
    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    // buf[6..7] 是温度，跳过
    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    // 单位转换: g → m/s², 原始值 → °/s
    const float g = 9.80665f;
    data->accel_x = (float)raw_ax / ACCEL_FS_SENSITIVITY * g;
    data->accel_y = (float)raw_ay / ACCEL_FS_SENSITIVITY * g;
    data->accel_z = (float)raw_az / ACCEL_FS_SENSITIVITY * g;
    data->gyro_x  = (float)raw_gx / GYRO_FS_SENSITIVITY;
    data->gyro_y  = (float)raw_gy / GYRO_FS_SENSITIVITY;
    data->gyro_z  = (float)raw_gz / GYRO_FS_SENSITIVITY;

    return ESP_OK;
}
