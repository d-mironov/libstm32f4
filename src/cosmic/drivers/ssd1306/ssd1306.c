#include <stm32f4xx.h>
#include <stdbool.h>
#include "../../i2c.h"
#include "ssd1306.h"


bool ssd1306_init(ssd1306 *self) {

    return true;
}


bool ssd1306_is_ok(ssd1306 *self) {
    return self->err == SSD1306_OK;
}
