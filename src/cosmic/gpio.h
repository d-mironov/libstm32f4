/*
 * Copyright [2022]
 * @author: Daniel Mironow
 * @file: gpio.h
 * Description:
 * GPIO driver abstraction header file
 *
 * Changelog v0.1:
 *
 * TODO:
 * -> Rewrite to new dir, speed,...
 * -> Rewrite new register abstractions
 */
#pragma once

#include <stdbool.h>
#include <stm32f4xx.h>

#include "cosmic.h"
#include "error.h"

#define NULL (void*)0x00

/* Values to write to GPIO */
#define GPIO_ON  0x01
#define GPIO_OFF 0x00

// How many pins there are per port
#define PINS_PER_PORT 0x10

/* Speeds of GPIO */
#define GPIO_LOW_SPEED    0x00
#define GPIO_MEDIUM_SPEED 0x01
#define GPIO_FAST_SPEED   0x02
#define GPIO_HIGH_SPEED   0x03

/* Pull up/down of GPIO */
#define GPIO_NO_PULL_UP_DOWN 0x00
#define GPIO_PULL_UP         0x01
#define GPIO_PULL_DOWN       0x02

// Push pull/Open drain
#define GPIO_OUT_PP 0x00
#define GPIO_OUT_OD 0x01

// Alternate function selection of GPIO
#define GPIO_AF00 0x00
#define GPIO_AF01 0x01
#define GPIO_AF02 0x02
#define GPIO_AF03 0x03
#define GPIO_AF04 0x04
#define GPIO_AF05 0x05
#define GPIO_AF06 0x06
#define GPIO_AF07 0x07
#define GPIO_AF08 0x08
#define GPIO_AF09 0x09
#define GPIO_AF10 0x0A
#define GPIO_AF11 0x0B
#define GPIO_AF12 0x0C
#define GPIO_AF13 0x0D
#define GPIO_AF14 0x0E
#define GPIO_AF15 0x0F

#define ADC_PA0 0x00
#define ADC_PA1 0x01
#define ADC_PA2 0x02
#define ADC_PA3 0x03
#define ADC_PA4 0x04
#define ADC_PA5 0x05
#define ADC_PA6 0x06
#define ADC_PA7 0x07
#define ADC_PB0 0x08
#define ADC_PB1 0x09
#define ADC_PC0 0x0A
#define ADC_PC1 0x0B
#define ADC_PC2 0x0C
#define ADC_PC3 0x0D
#define ADC_PC4 0x0E
#define ADC_PC5 0x0F

typedef enum pin_mode {
    GPIO_INPUT,
    GPIO_OUTPUT,
    GPIO_ALTERNATE,
    GPIO_ANALOG,
    GPIO_OUTPUT_PULLUP,
    GPIO_OUTPUT_PULLDOWN,
} pin_mode_t;

typedef enum _pindir { INPUT, OUTPUT, ALTERNATE, ANALOG } pindir_t;

typedef enum _outtype { PUSHPULL, OPENDRAIN } pin_outtype_t;

typedef enum _pinspeed {
    LOW_SPEED,
    MEDIUM_SPEED,
    FAST_SPEED,
    HIGH_SPEED
} pinspeed_t;
typedef enum _pinpull { NOPULL, PULLUP, PULLDOWN } pinpull_t;

// typedef enum pin_err {
//     GPIO_OK,
//     GPIO_PIN_TOO_HIGH,
//     GPIO_ALTERNATE_FUNC_TOO_HIGH,
//     GPIO_ALTERNATE_NOT_SELECTED,
//     GPIO_INVALID_SETTING
// } pin_err_t;

/**
 * GPIO pin definitions
 */
typedef enum _pinnum_t {
    // GPIOA pins
    PA0,
    PA1,
    PA2,
    PA3,
    PA4,
    PA5,
    PA6,
    PA7,
    PA8,
    PA9,
    PA10,
    PA11,
    PA12,
    PA13,
    PA14,
    PA15,
    // GPIOB pins
    PB0,
    PB1,
    PB2,
    PB3,
    PB4,
    PB5,
    PB6,
    PB7,
    PB8,
    PB9,
    PB10,
    PB11,
    PB12,
    PB13,
    PB14,
    PB15,
    // GPIOC pins
    PC0,
    PC1,
    PC2,
    PC3,
    PC4,
    PC5,
    PC6,
    PC7,
    PC8,
    PC9,
    PC10,
    PC11,
    PC12,
    PC13,
    PC14,
    PC15,
    // GPIOD pins
    PD0,
    PD1,
    PD2,
    PD3,
    PD4,
    PD5,
    PD6,
    PD7,
    PD8,
    PD9,
    PD10,
    PD11,
    PD12,
    PD13,
    PD14,
    PD15,
    // GPIOE pins
    PE0,
    PE1,
    PE2,
    PE3,
    PE4,
    PE5,
    PE6,
    PE7,
    PE8,
    PE9,
    PE10,
    PE11,
    PE12,
    PE13,
    PE14,
    PE15,
    // GPIOH pins
    PH0,
    PH1,
    PH2,
    PH3,
    PH4,
    PH5,
    PH6,
    PH7,
    PH8,
    PH9,
    PH10,
    PH11,
    PH12,
    PH13,
    PH14,
    PH15,
    // all from port
    PA,
    PB,
    PC,
    PD,
    PE,
    PH,
} pinnum_t;

error pin_enable(const pinnum_t, pin_mode_t mode);

error pin_settings(const pinnum_t, const u8 speed, const u8 pull_up_down,
                   const u8 push_pull_open_drain);

error pin_set_speed(const pinnum_t, const u8 speed);
error pin_set_pull_up_down(const pinnum_t pin, const u8 pull_up_down);

error pin_toggle(const pinnum_t pin);
error pin_write(const pinnum_t pin, const u8 high_low);
error pin_high(const pinnum_t pin);
error pin_low(const pinnum_t pin);

u8 pin_read(const pinnum_t pin);
u8 pin_value(const pinnum_t pin);
bool pin_is_high(const pinnum_t pin);
error pin_select_alternate(const pinnum_t pin, const u8 af);

error pin_lock(const pinnum_t pin);

// TODO: Write functions
error pin_make_output(const pinnum_t);
error pin_make_input(const pinnum_t);
error pin_make_analog(const pinnum_t);
error pin_make_alternative(const pinnum_t);

GPIO_TypeDef* _GPIO_fetch_port(const pinnum_t pin);

// TODO: implement functions

// gpio attach external interrupt
error pin_attach_exti(const pinnum_t pin, void*(func)(void));

//#endif
