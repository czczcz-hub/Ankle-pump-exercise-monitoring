#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "mpu6050.h"
#include "oled.h"
#include "filter.h"
#include "nvs_flash.h"
#include "shared_data.h"
#include "ble_service.h"

#define MA_WINDOW_SIZE  10

static void sensor_task(void *pvParameters)
{
    mpu6050_data_t data;
    int8_t   zone = 0;
    uint32_t count_dorsi   = 0;
    uint32_t count_plantar = 0;
    uint32_t event_id      = 0;

    float ma_ay_buf[MA_WINDOW_SIZE], ma_az_buf[MA_WINDOW_SIZE];
    float ma_gx_buf[MA_WINDOW_SIZE], ma_gy_buf[MA_WINDOW_SIZE], ma_gz_buf[MA_WINDOW_SIZE];
    comp_filter_t filter;
    comp_filter_init(&filter, 0.98f,
                     ma_ay_buf, ma_az_buf,
                     ma_gx_buf, ma_gy_buf, ma_gz_buf, MA_WINDOW_SIZE);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (xSemaphoreTake(g_state.i2c_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            mpu6050_read(&data);
            xSemaphoreGive(g_state.i2c_mutex);
        } else {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
            continue;
        }

        float angle = comp_filter_update(&filter,
                                         data.accel_y, data.accel_z,
                                         data.gyro_x, data.gyro_y, data.gyro_z,
                                         0.02f);

        movement_zone_t new_zone = ZONE_IDLE;
        bool event_occurred = false;

        if (angle > DORSI_THRESHOLD) {
            new_zone = ZONE_DORSIFLEX;
            if (zone != ZONE_DORSIFLEX) {
                count_dorsi++;
                event_occurred = true;
                printf("Dorsiflex #%lu (angle=%.1f)\n", count_dorsi, angle);
            }
        } else if (angle < -PLANTAR_THRESHOLD) {
            new_zone = ZONE_PLANTARFLEX;
            if (zone != ZONE_PLANTARFLEX) {
                count_plantar++;
                event_occurred = true;
                printf("Plantarflex #%lu (angle=%.1f)\n", count_plantar, angle);
            }
        }
        zone = new_zone;

        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        g_state.angle         = angle;
        g_state.gyro_x        = filter.gyro_x;
        g_state.gyro_y        = filter.gyro_y;
        g_state.gyro_z        = filter.gyro_z;
        g_state.accel_x       = data.accel_x;
        g_state.accel_y       = data.accel_y;
        g_state.accel_z       = data.accel_z;
        g_state.zone          = zone;
        g_state.count_dorsi   = count_dorsi;
        g_state.count_plantar = count_plantar;

        if (event_occurred) {
            g_state.ble_event.angle        = angle;
            g_state.ble_event.zone         = zone;
            g_state.ble_event.count_dorsi  = count_dorsi;
            g_state.ble_event.count_plantar= count_plantar;
            g_state.ble_event.event_id     = ++event_id;
        }
        xSemaphoreGive(g_state.mutex);

        if (event_occurred) {
            xSemaphoreGive(g_state.ble_sem);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}

static void display_task(void *pvParameters)
{
    printf("[DISP] started, prio=%d\n", uxTaskPriorityGet(NULL));
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t loop = 0;

    while (1) {
        float    angle; float gx, gy, gz;
        uint32_t cd, cp;
        int8_t   zone;

        if (xSemaphoreTake(g_state.mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
            continue;
        }
        angle = g_state.angle;
        gx    = g_state.gyro_x;
        gy    = g_state.gyro_y;
        gz    = g_state.gyro_z;
        cd    = g_state.count_dorsi;
        cp    = g_state.count_plantar;
        zone  = g_state.zone;
        xSemaphoreGive(g_state.mutex);

        const char *status = (zone == ZONE_DORSIFLEX)  ? "Dorsiflex" :
                             (zone == ZONE_PLANTARFLEX) ? "Plantarflex" : "Idle";

        if (xSemaphoreTake(g_state.i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            char line[32];
            snprintf(line, sizeof(line), "Angle: %.1f", angle);
            oled_show_text(2, line);
            snprintf(line, sizeof(line), "G:%.1f %.1f %.1f", gx, gy, gz);
            oled_show_text(3, line);
            snprintf(line, sizeof(line), "D:%lu P:%lu", cd, cp);
            oled_show_text(4, line);
            snprintf(line, sizeof(line), "Status: %s", status);
            oled_show_text(5, line);
            xSemaphoreGive(g_state.i2c_mutex);
            if (loop % 50 == 0) printf("[DISP] loop=%lu ok\n", loop);
        } else {
            if (loop % 10 == 0) printf("[DISP] i2c timeout loop=%lu\n", loop);
        }

        loop++;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}

static void ble_notify_task(void *pvParameters)
{
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        float    angle, gx, gy, gz, ax, ay, az;
        int8_t   zone;
        uint32_t cd, cp;
        uint32_t now_ms;

        xSemaphoreTake(g_state.mutex, portMAX_DELAY);
        angle  = g_state.angle;
        gx     = g_state.gyro_x;
        gy     = g_state.gyro_y;
        gz     = g_state.gyro_z;
        ax     = g_state.accel_x;
        ay     = g_state.accel_y;
        az     = g_state.accel_z;
        zone   = g_state.zone;
        cd     = g_state.count_dorsi;
        cp     = g_state.count_plantar;
        now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
        xSemaphoreGive(g_state.mutex);

        cJSON *root = cJSON_CreateObject();
        cJSON_AddNumberToObject(root, "ts",    now_ms);
        cJSON_AddNumberToObject(root, "angle", (double)angle);
        cJSON_AddNumberToObject(root, "gx",    (double)gx);
        cJSON_AddNumberToObject(root, "gy",    (double)gy);
        cJSON_AddNumberToObject(root, "gz",    (double)gz);
        cJSON_AddNumberToObject(root, "ax",    (double)ax);
        cJSON_AddNumberToObject(root, "ay",    (double)ay);
        cJSON_AddNumberToObject(root, "az",    (double)az);
        cJSON_AddStringToObject(root, "move",
            zone == ZONE_DORSIFLEX  ? "d" :
            zone == ZONE_PLANTARFLEX ? "p" : "i");
        cJSON_AddNumberToObject(root, "cnt_d", cd);
        cJSON_AddNumberToObject(root, "cnt_p", cp);

        char *json_str = cJSON_PrintUnformatted(root);
        if (json_str) {
            ble_notify_movement(json_str, strlen(json_str));
            cJSON_free(json_str);
        }
        cJSON_Delete(root);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    shared_state_init();

    if (mpu6050_init() != ESP_OK) {
        printf("MPU6050 init failed, check wiring.\n");
        return;
    }

    if (oled_init() != ESP_OK) {
        printf("OLED init failed, check wiring.\n");
        return;
    }

    oled_clear();
    oled_show_text(0, "Ankle Pump Monitor");
    oled_show_text(1, "Starting BLE...");

    ble_service_init();
    oled_show_text(1, "BLE: AnklePump");

    xTaskCreate(sensor_task,     "sensor",     4096, NULL, 2, NULL);
    xTaskCreate(display_task,    "display",    3072, NULL, 2, NULL);
    xTaskCreate(ble_notify_task, "ble_notify", 4096, NULL, 2, NULL);
}
