#include <stm32f4xx.h>
#include "../error.h"
#include <stdbool.h>

bool is_ok(error err) {
    return err == OK;
}
