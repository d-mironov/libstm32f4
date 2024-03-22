#include "../rcc.h"
#include <stm32f4xx.h>
#include "../pwr.h"

u32 SYSTEM_CLOCK = 16000000;
u32 apb1_freq = 16000000;
u32 apb2_freq = 16000000;

void rcc_periphclock_enable(rcc_clock_port_t port, u32 periph_enable, u8 enable) {
    if (enable == RCC_ENABLE) {
        if (port == RCC_AHB1) {
            RCC->AHB1ENR |= periph_enable;
        } else if (port == RCC_AHB2) {
            RCC->AHB2ENR |= periph_enable;
        } else if (port == RCC_APB1) {
            RCC->APB1ENR |= periph_enable;
        } else if (port == RCC_APB2) {
            RCC->APB2ENR |= periph_enable;
        }
    } else {
        if (port == RCC_AHB1) {
            RCC->AHB1RSTR |= periph_enable;
        } else if (port == RCC_AHB2) {
            RCC->AHB2RSTR |= periph_enable;
        } else if (port == RCC_APB1) {
            RCC->APB1RSTR |= periph_enable;
        } else if (port == RCC_APB2) {
            RCC->APB2RSTR |= periph_enable;
        } 
    } 
}


/**
 * Sets the PLL M prescaler
 */
void rcc_set_pllm_pre(u32 pll_m) {
    RCC->PLLCFGR |= (pll_m << RCC_PLL_M_OFFSET);
}
/**
 * Sets the PLL N prescaler
 */
void rcc_set_plln_pre(u32 pll_n) {
    RCC->PLLCFGR |= (pll_n << RCC_PLL_N_OFFSET);
}
/**
 * Sets the PLL P prescaler
 */
void rcc_set_pllp_pre(u32 pll_p) {
    RCC->PLLCFGR |= pll_p;
}
/**
 * Sets the PLL Q prescaler
 */
void rcc_set_pllq_pre(u32 pll_q) {
    RCC->PLLCFGR |= pll_q;
}

void rcc_set_osc(rcc_osc_t osc) {
    RCC->CR |= osc;
}

void rcc_reset_osc(rcc_osc_t osc) {
    RCC->CR &= ~osc;
}

bool rcc_osc_rdy(rcc_osc_t osc) {
    switch (osc) {
        case RCC_OSC_HSI:
            return RCC->CR & RCC_HSIRDY;
        case RCC_OSC_HSE:
            return RCC->CR & RCC_HSERDY;
        case RCC_OSC_PLL:
            return RCC->CR & RCC_PLLRDY;
        default:
            return RCC->CR & RCC_HSIRDY;
    }
}


void rcc_wait_osc_rdy(volatile rcc_osc_t osc) {
    while (!(rcc_osc_rdy(osc)));
}

void rcc_set_sysclk_src(volatile rcc_sysclksrc_t src) {
    RCC->CFGR |= src;
}


void rcc_set_pll_src(volatile rcc_pllsrc_t src) {
    RCC->PLLCFGR |= (src << RCC_PLLSRC_OFFSET);
}


void rcc_system_clock_config(clock_t clock) {
    //RCC->PLLCFGR = 0x24003010;
    rcc_set_osc(RCC_OSC_HSI);
    rcc_wait_osc_rdy(RCC_OSC_HSI);
    
    rcc_set_sysclk_src(RCC_SYSCLK_HSI);

    if (clock.pll_src == RCC_PLLSRC_HSE) {
        rcc_set_osc(RCC_OSC_HSE);
        rcc_wait_osc_rdy(RCC_OSC_HSE);
    }

    rcc_periphclock_enable(RCC_APB1, RCC_APB1_PWR, RCC_ENABLE);
    pwr_set_voltage_scaling(PWR_SCALE_1); 


    rcc_set_ahb_pre(clock.ahb_pre);
    rcc_set_apb1_pre(clock.apb1_pre);
    rcc_set_apb2_pre(clock.apb2_pre);

    rcc_reset_osc(RCC_OSC_PLL);

    FLASH->ACR = (1<<8) | (1<<9) | (1<<10) | (5<<0);

    if (clock.pll_src == RCC_PLLSRC_HSE) {
        rcc_set_pllq_pre(clock.pll_q);
        rcc_set_pllp_pre(clock.pll_p);
        rcc_set_plln_pre(clock.pll_n);
        rcc_set_pllm_pre(clock.pll_m);
        rcc_set_pll_src(clock.pll_src);
    } else {

    }

    rcc_set_osc(RCC_OSC_PLL);
    rcc_wait_osc_rdy(RCC_OSC_PLL);
    rcc_set_sysclk_src(RCC_SYSCLK_PLL);
    rcc_wait_sysclk_rdy(RCC_SYSCLK_PLL);

    SYSTEM_CLOCK = clock.ahb_freq;
    apb1_freq = clock.apb1_freq;
    apb2_freq = clock.apb2_freq;

    rcc_reset_osc(RCC_OSC_HSI);
    SystemCoreClockUpdate();
}


void rcc_set_ahb_pre(volatile rcc_ahb_pre_t ahb_pre) {
    RCC->CFGR |= ahb_pre; 
}

void rcc_set_apb1_pre(volatile rcc_apb1_pre_t apb1_pre) {
    RCC->CFGR |= apb1_pre;
}

void rcc_set_apb2_pre(volatile rcc_apb2_pre_t apb2_pre) {
    RCC->CFGR |= apb2_pre;
}

void rcc_reset_ahb_pre(volatile rcc_ahb_pre_t ahb_pre) {
    RCC->CFGR &= ~ahb_pre; 
}

void rcc_reset_apb1_pre(volatile rcc_apb1_pre_t apb1_pre) {
    RCC->CFGR &= ~apb1_pre;
}

void rcc_reset_apb2_pre(volatile rcc_apb2_pre_t apb2_pre) {
    RCC->CFGR &= ~apb2_pre;
}

bool rcc_sysclk_rdy(volatile rcc_sysclksrc_t src) {
    return RCC->CFGR & (src << 2);
}

void rcc_wait_sysclk_rdy(volatile rcc_sysclksrc_t src) {
    while(!(rcc_sysclk_rdy(src)));
}
