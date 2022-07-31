#include <stm32f4xx.h>
#include "../cosmic.h"
#include "../spi.h"


bool spi_init(spi *self) {

    return true;
}


bool spi_is_ok(spi *self) {
    return self->err == OK;
}
