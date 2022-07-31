#ifndef _COSMIC_BITUTILS_H
#define _COSMIC_BITUTILS_H

#include "types.h"
#include <stdbool.h>

#define setbit_var(var,y)           (var |= (1 << (y)))         /*!< Set bit `y` of variable `x` */
#define setbit(x)                   (1 << (x))                  /*!< Set bit `x` */
#define setbits_var(var, bits, n)   (var |= (bits << (n)))
#define setbits(var, bits, n)       (bits << (n))
#define bit_is_set(var, pos)        ((var) & (1 << (pos)))   
#define bit_clear(var, pos)         ((var) &= ~(1 << (pos)))

#define inc(var, n)             ((var) += n)


extern u32 _bit[32];
#define bit(n) _bit[n]

inline void reg_modify(u32* reg, u32 val) {
    *reg |= val;
}

inline void reg_write(u32* reg, u32 val) {
    *reg = val;
}

inline u32 reg_read(u32* reg) {
    return *reg;
}

inline bool reg_is_eq(u32* reg, u32 val) {
    return *reg == val;
}

inline bool reg_is_set(u32* reg, u8 n) {
    return *reg & bit(n);
}

#endif
