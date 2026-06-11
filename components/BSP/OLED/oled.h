#pragma once

#include "esp_err.h"

#define OLED_I2C_ADDR           0x3C
#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_PAGES              8
#define OLED_CHARS_PER_LINE     21

esp_err_t oled_init(void);
void oled_clear(void);
void oled_clear_line(uint8_t line);
void oled_show_text(uint8_t line, const char *text);
void oled_show_text_xy(uint8_t x, uint8_t page, const char *text);
