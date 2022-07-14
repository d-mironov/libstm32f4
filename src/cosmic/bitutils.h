#ifndef _COSMIC_BITUTILS_H
#define _COSMIC_BITUTILS_H

#define setbit_var(var,y)           (var |= (1 << (y)))         /*!< Set bit `y` of variable `x` */
#define setbit(x)                   (1 << (x))                  /*!< Set bit `x` */
#define setbits_var(var, bits, n)   (var |= (bits << (n)))
#define setbits(var, bits, n)       (bits << (n))
#define bit_is_set(var, pos)        ((var) & (1 << (pos)))   
#define bit_clear(var, pos)         ((var) &= ~(1 << (pos)))

#define inc(var, n)             ((var) += n)

#endif
