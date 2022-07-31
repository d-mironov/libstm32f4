#ifndef _COSMIC_H
#define _COSMIC_H

#include <stdint.h> 
#include "error.h"
#include "bitutils.h"
#include "types.h"

#define and     &&
#define or      ||

/**
 * Create string with size `n`
 */
#define new_str(n)          (char[(n)]){}
#define v3(a, b, c)         (vec3){a, b, c}
#define v4(a, b, c, d)      (vec4){a, b, c, d}

#define v3_new()            (vec3){0, 0, 0}
#define v4_new()            (vec4){0, 0, 0, 0}

#define VEC_DEC_STR         2

#define io                  volatile

typedef enum {
    type_error, type_usart, type_i2c, type_timer,
    type_u8, type_u16, type_u32, type_u64,
    type_i8, type_i16, type_i32, type_i64,
    type_f32, type_f64, type_vec3, type_vec4,
    type_str, type_undefined,
} types_t;

/**
 * @brief Cast type of `x` to `type`
 */
#define as(typ, x)          ((typ)(x))      /*!< cast `x` to `type`*/

/**
 * @brief Get `x` as pointer 
 */
#define as_ptr(x)           (&x)            /*!< get `x` as pointer */

#define deref(x)            (*x)


//#define     u8      uint8_t

// ======| unsigned int |======




/**
 * 3D Vector
 */
typedef struct _vec3 {
    f32     x;
    f32     y;
    f32     z;
} vec3;

/**
 * 16-bit signed integer 3D vector
 */
typedef struct _vec3_i16 {
    i16     x;
    i16     y;
    i16     z;
} vec3_i16;

/**
 * 4D Vector
 */
typedef struct _vec4 {
    f32     x;
    f32     y;
    f32     z;
    f32     w;
} vec4;

typedef struct _tuple {
    f32     $1;
    f32     $2;
    f32     $3;
    f32     $4;
    f32     $5;
    f32     $6;
    f32     $7;
    f32     $8;
    f32     $9;
    f32     $10;
} tuple;

vec3 v3_add(const vec3 v1, const vec3 v2);
vec4 v4_add(const vec4 v1, const vec4 v2);

vec3 v3_smult(const vec3 v1, const f32 s);
vec4 v4_smult(const vec4 v1, const f32 s);
i64  v3_mult(const vec3 v1, const vec3 v2);
i64  v4_mult(const vec4 v1, const vec4 v2);
vec3 v3_id(const vec3 v1, const vec4);
vec4 v4_id(const vec4 v1, const vec3);
void v3_to_string(const vec3, str buf);
void v4_to_string(const vec4, str buf);



/**
 * Vector multiplication
 */
#define vec_mult(x,y) _Generic((x),\
        vec3: _Generic((y),\
                vec3:   v3_mult,\
                default: v3_smult),\
        vec4: _Generic((y),\
                vec4:   v4_mult,\
                default: v4_smult)\
)(x,y)

/**
 * Output vector as string
 * @param vector - `vec3` or `vec4`
 * @param buf - Allocated String buffer
 *
 * @return vector as string
 */
#define vec_to_str(vector,buf) _Generic((vector),\
        vec3: v3_to_string,\
        vec4: v4_to_string\
)(vector,buf)

#define vec_eq(v1,v2) _Generic((v1),\
        vec3: v3_eq,\
        vec4: v4_eq\
)(v1,v2)


//============| str utils |=============

#define MAX_SLICE_SIZE  512


void str_slice(str in, str buf, u32 start, u32 stop);


#endif
