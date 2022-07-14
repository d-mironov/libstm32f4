#pragma once
#include <stm32f4xx.h>
#include <stdbool.h>
#include "cosmic.h"


typedef struct _spi {
    SPI_TypeDef *port;
    error err;
} spi;


bool spi_init(spi *self);
bool spi_is_ok(spi *self);
