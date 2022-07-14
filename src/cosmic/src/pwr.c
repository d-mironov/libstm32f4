#include "../pwr.h"
#include "../gpio.h"
#include "../rcc.h"


void pwr_set_voltage_scaling(pwr_vscaling_t scaling) {
    PWR->CR |= scaling;
}

void pwr_reboot() {
    NVIC_SystemReset();
}
