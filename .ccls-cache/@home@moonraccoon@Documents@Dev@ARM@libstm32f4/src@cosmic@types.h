#pragma once
#include "stm32f4xx.h"

#define or ||
#define and &&

/**
 * 8-bit unsigned integer */
typedef uint8_t         u8;
/**
 * 16-bit unsigned integer
 */
typedef uint16_t        u16;
/**
 * 32-bit unsigned integer
 */
typedef uint32_t        u32;

/**
 * 64-bit unsigned integer
 */
typedef uint64_t        u64;

// ======| signed int |======

/**
 * 8-bit signed integer
 */
typedef int8_t          i8;
/**
 * 16-bit signed integer
 */
typedef int16_t         i16;
/**
 * 32-bit signed integer
 */
typedef int32_t         i32;
/**
 * 64-bit signed integer
 */
typedef int64_t         i64;

// ======| floats |======

/**
 * 32-bit float
 */
typedef float           f32;
/**
 * 64-bit float
 */
typedef double          f64;

/**
 * String
 */
typedef char*           str;

/**
 * Byte
 */
typedef uint8_t         byte;

#define type_str(x)     _Generic((x),\
        u8:     "u8",   u16:    "u16",\
        u32:    "u32",  u64:    "u64",\
        i8:     "i8",   i16:    "i16",\
        i32:    "i32",  i64:    "i64",\
        f32:    "f32",  f64:    "f64",\
        vec3:   "vec3", vec4:   "vec4",\
        str:    "str",\
        default: "undefined"\
)
// GET TYPE
#define type(x)     _Generic((x),\
        u8:     type_u8,        u16:    type_u16,\
        u32:    type_u32,       u64:    type_u64,\
        i8:     type_i8,        i16:    type_i16,\
        i32:    type_i32,       i64:    type_i64,\
        f32:    type_f32,       f64:    type_f64,\
        vec3:   type_vec3,      vec4:   type_vec4,\
        str:    type_str,       \
        default:type_undefined\
)
