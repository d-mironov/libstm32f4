#include "../exti.h"
#include "../gpio.h"
#include "../uart.h"
#include <stm32f4xx.h>



error exti_select_trigger(u32 lines, exti_trigger_t trigger) {
    if (lines & EXTI_RESERVED) {
        return EXTI_LINES_RESERVED;
    }

    switch (trigger) {
        case EXTI_RISING_EDGE:
            EXTI->RTSR &= ~(lines);
            EXTI->RTSR |= lines;
            break;
        case EXTI_FALLING_EDGE:
            EXTI->FTSR &= ~(lines);
            EXTI->FTSR |= lines;
            break;
        default:
            EXTI->RTSR &= ~(lines);
            EXTI->FTSR &= ~(lines);
            EXTI->RTSR |= lines;
            EXTI->FTSR |= lines;
            break;
    }

    return OK;
}



error exit_unmask(u32 lines) {
    if (lines & EXTI_RESERVED) {
        return EXTI_LINES_RESERVED;
    }

    EXTI->IMR |= lines;

    return OK;
}


error exti_nvic_enable_irq(u8 pin) {
    if (pin == 0) {
        NVIC_EnableIRQ(EXTI0_IRQn);
    } else if (pin == 1) {
        NVIC_EnableIRQ(EXTI1_IRQn);
    } else if (pin == 2) {
        NVIC_EnableIRQ(EXTI2_IRQn);
    } else if (pin == 3) {
        NVIC_EnableIRQ(EXTI3_IRQn);
    } else if (pin == 4) {
        NVIC_EnableIRQ(EXTI4_IRQn);
    } else if (pin >= 5 && pin <= 9) {
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    } else if (pin >= 10 && pin <= 15) {
        NVIC_EnableIRQ(EXTI15_10_IRQn);
    } else {
        return EXTI_PIN_TOO_HIGH;
    }
    return OK;
}


error exti_attach_gpio(const pinnum_t pin, exti_trigger_t trigger) {
    if (pin > 15) {
        return EXTI_PIN_TOO_HIGH;
    }

    __disable_irq();
    
    pin_enable(pin, GPIO_INPUT);
    GPIO_TypeDef *port = _GPIO_fetch_port(pin);
    if (port == NULL) {
        return EXTI_PIN_TOO_HIGH;
    }
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // u8 cr = (pin / 4) % 4;
    u8 exti_port = ((uint64_t) port - AHB1PERIPH_BASE) / 0x00400UL;
    // Calculations for port and pin on the EXTI line selection
    // Don't try to understand this, if you don't want you mind to blow up lol
    SYSCFG->EXTICR[(pin/4) % 4] |= (exti_port << ((pin % SYSCFG_EXTI_PORTS_PER_REG) * SYSCFG_EXTI_BITNUM)); 
    
    if ( exti_unmask( (1 << pin) ) != OK) {
        __enable_irq();
        return EXTI_LINES_RESERVED;
    }

    if ( exti_select_trigger((1<<pin), trigger) != OK ) {
        __enable_irq();
        return EXTI_LINES_RESERVED;
    }

    exti_nvic_enable_irq(pin);

    __enable_irq();
    return OK;
}


