#pragma once
#include <stddef.h>

void ble_service_init(void);

void ble_notify_movement(const char *json_str, size_t len);
