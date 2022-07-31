#include "../cosmic.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

vec3 v3_add(const vec3 v1, const vec3 v2) {
    vec3 out = {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z};
    return out;    
}

vec4 v4_add(const vec4 v1, const vec4 v2) {
    vec4 out = {v1.x + v2.x, v1.y + v2.y, v1.z + v2.z, v1.w + v2.w};
    return out;    
}

/**
 * Scalar multiplication on 3D vector
 * @param v1 => Vector to multiply
 * @param s  => Scalar
 *
 * @return vector after multiplication
 */
vec3 v3_smult(const vec3 v1, const f32 s) {
    vec3 out = {v1.x * s, v1.y * s, v1.z * s};
    return out;
}

/**
 * Scalar multiplication on 4D vector
 * @param v1 => Vector to multiply
 * @param s  => Scalar
 *
 * @return vector after multiplication
 */
vec4 v4_smult(const vec4 v1, const f32 s) {
    vec4 out = {v1.x * s, v1.y * s, v1.z * s, v1.w * s};
    return out;
}

/**
 * Dot multiplication on two 3D vectors
 * @param v1 => first vector
 * @param v2 => second vector
 *
 * @return dot multiplication
 */
i64 v3_mult(const vec3 v1, const vec3 v2) {
    i64 out = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    return out;    
}

/**
 * Dot multiplication on two 4D vectors
 * @param v1 => first vector
 * @param v2 => second vector
 *
 * @return dot multiplication
 */
i64 v4_mult(const vec4 v1, const vec4 v2) {
    i64 out = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z + v1.w * v2.w;
    return out;    
}

/**
 * Check if two 3D vectors are equal
 * @param v1 => First vector
 * @param v2 => Second vector
 *
 * @return true if equal, else false
 */
bool v3_eq(const vec3 v1, const vec3 v2) {
    return (v1.x == v2.x) and (v1.y == v2.y) and (v1.z == v2.z);
}

/**
 * Check if two 4D vectors are equal
 * @param v1 => First vector
 * @param v2 => Second vector
 *
 * @return true if equal, else false
 */
bool v4_eq(const vec4 v1, const vec4 v2) {
    return (v1.x == v2.x) and (v1.y == v2.y) and (v1.z == v2.z) and (v1.w == v2.w);
}

vec3 v3_id(const vec3 v, const vec4 null) {
    return (vec3){v.x, v.y, v.z};
}

vec4 v4_id(const vec4 v, const vec3 null) {
    return (vec4){v.x, v.y, v.z, v.w};
}

/**
 * Output 3D vector as string
 * @param v - vector to convert
 * @param buf - allocated string buffer
 *
 * @return vector as string
 */
void v3_to_string(const vec3 v, str buf) {
    sprintf(buf, "{\n\t.x = %.2f,\n\t.y = %.2f,\n\t.z = %.2f\n}", v.x, v.y, v.z);
}

/**
 * Output 4D vector as string
 * @param v - vector to convert
 * @param buf - allocated string buffer
 *
 * @return vector as string
 */
void v4_to_string(const vec4 v, str buf) {
    sprintf(buf, "{\n\t.x = %.2f,\n\t.y = %.2f,\n\t.z = %.2f,\n\t.w = %.2f}\n", v.x, v.y, v.z, v.w);
}

//================| str utils |================
void str_slice(str in, str buf, u32 start, u32 stop) {
    if (start > stop) {
        return;
    }
    i32 len = strlen(in);
    if (start > len or stop > len) {
        return;
    }
    u32 i;
    for (i = start; i < stop; inc(i, 1)) {
        buf[i - start] = in[i];
    }
    buf[i - start] = '\0';
}

