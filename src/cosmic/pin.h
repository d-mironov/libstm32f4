#pragma once
#include "bitutils.h"
#include "types.h"

typedef enum _pindir { INPUT, OUTPUT, ALTERNATE, ANALOG } pindir_t;

typedef enum _outtype { PUSHPULL, OPENDRAIN } pin_outtype_t;

typedef enum _pinspeed { LOW, MEDIUM, FAST, HIGH } pinspeed_t;
typedef enum _pinpull { NOPULL, PULLUP, PULLDOWN } pinpull_t;

typedef enum _pin_af {
    AF0,
    AF1,
    AF2,
    AF3,
    AF4,
    AF5,
    AF6,
    AF7,
    AF8,
    AF9,
    AF10,
    AF11,
    AF12,
    AF13,
    AF14,
    AF15,
} pin_af_t;

typedef struct _pin {
    // public
    u32 pin;
    pindir_t dir;
    pin_outtype_t outtype;
    pinspeed_t speed;
    pinpull_t pull;
    pin_af_t af;
    // private
    GPIO_TypeDef* __port;
} pin_t;

void pin_clock_enable(pin_t* pin);

void pin_mode(pin_t* pin, pindir_t dir);
void pin_make_output(pin_t* pin);
void pin_make_input(pin_t* pin);
void pin_make_alternate(pin_t* pin);
void pin_make_analog(pin_t* pin);

void pin_outtype(pin_t* pin, pin_outtype_t otype);
void pin_pushpull(pin_t* pin);
void pin_opendrain(pin_t* pin);

void pin_pull(pin_t* pin, pinpull_t pull);
void pin_nopull(pin_t* pin);
void pin_pullup(pin_t* pin);
void pin_pulldown(pin_t* pin);

void pin_af(pin_t* pin, pin_af_t af);

void pin_set_value(pin_t* pin, u8 val);
bool pin_value(pin_t* pin);

void pin_lock(pin_t* pin);
