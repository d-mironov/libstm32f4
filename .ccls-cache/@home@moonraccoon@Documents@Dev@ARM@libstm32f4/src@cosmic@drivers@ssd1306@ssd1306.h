#pragma once
#include <stm32f4xx.h>
#include <stdbool.h>
#include "../../i2c.h"
#include "../../cosmic.h"

#define SSD1306_ADDR_DEFAULT        (0x00)


typedef enum _ssd1306_err {
    SSD1306_OK,
} ssd1306_err;


typedef struct _ssd1306 {
    i2c *i2c;
    i32 x;
    i32 y;
    ssd1306_err err;
} ssd1306;



bool ssd1306_init(ssd1306 *self);

bool ssd1306_is_ok(ssd1306 *self);



