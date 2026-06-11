#include "shared_data.h"
#include <string.h>

shared_state_t g_state;

void shared_state_init(void)
{
    memset(&g_state, 0, sizeof(g_state));
    g_state.mutex     = xSemaphoreCreateMutex();
    g_state.ble_sem   = xSemaphoreCreateBinary();
    g_state.i2c_mutex = xSemaphoreCreateMutex();
}
