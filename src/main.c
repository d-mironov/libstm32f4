// Copyright [2022] <Daniel Mironow>
#include <stdbool.h>
#include <stm32f4xx.h>

#include "cosmic/bitutils.h"
#include "cosmic/cosmic.h"
#include "cosmic/delay.h"
#include "cosmic/drivers/bno/bno.h"
#include "cosmic/gpio.h"
#include "cosmic/i2c.h"
#include "cosmic/pwr.h"
#include "cosmic/rcc.h"
#include "cosmic/timer.h"
#include "cosmic/uart.h"

#define DEBUG_LED PB8
#define TEST_LED  PA8

// peripheral structure
bno055 bno;
i2c i2c1;
usart port;
timer tim5;

// error values
error usart_err, err_tim, i2c_err;
error_bno bno_err;

// IMU sensor data vectors
vec3 gyro, acc, mag, euler, lia, gravity;
vec4 quat;

// Misc variables
i8 temperature;
u8 addr_data;
u32 min = 0, hour = 0;
u64 cycle = 0;
/**
 * Test function for timer interrupt
 */
void toggle_test_led(void) {
    pin_toggle(DEBUG_LED);
    inc(cycle, 1);
}

/**
 * Main execution function
 */
int main(void) {
    // Set system clock to 96MHz
    rcc_system_clock_config(rcc_hse_25_mhz_to_96_mhz);
    // I2C1 init object
    i2c1 = (i2c){
        .i2c = I2C1,
        .frequency = 16,
        .mode = I2C_STD_MODE,
        .duty = 0,
    };
    // USART2 init object
    port = (usart){
        .usart = USART2,
        .baud = 115200,
        .mode = USART_RX_TX_MODE,
        .stop_bits = 0,
        .parity_enable = 0,
        .parity_even_odd = 0,
        .interrupt_driven = true,
    };
    // Timer 5 init object
    tim5 = (timer){
        .timer = TIM5,
        .prescaler = (apb1_freq * 2) / 10000,
        .autoreload = 10000,
        .func = toggle_test_led,
        .interrup_en = true,
    };
    // enable GPIO output
    pin_enable(DEBUG_LED, GPIO_OUTPUT);
    pin_enable(PA8, GPIO_OUTPUT);

    //========================| Initialization |========================
    usart_init(as_ptr(port));
    // USART_init(&gps);
    usart_printf(port, "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
    delay_ms(1000);
    i2c_init(as_ptr(i2c1));

    // BNO
    bno = (bno055){
        .i2c = &i2c1,
        .mode = BNO_MODE_IMU,
    };
    // i2c_err = I2C_write(i2c1, MPU_ADDR, 0x6B, 0x00);
    usart_printf(port, "%s\n\n", i2c_get_err_str(i2c_err));
    delay_ms(1000);
    // initialize BNO sensor
    if (bno055_init(as_ptr(bno))) {
        usart_printf(port, "[BNO] init success!\n");
    } else {
        usart_printf(port, "[BNO] init failed\n");
    }
    delay_ms(1000);
    str post_result = new_str(255);
    bno055_post_result(as_ptr(bno), post_result);
    usart_printf(port, "[BNO] Self test result:\n%s", post_result);
    // set accelerometer config
    bno.err = bno055_set_acc_conf(as_ptr(bno), BNO_ACC_RANGE_4G,
                                  BNO_ACC_BAND_250, BNO_ACC_MODE_NORMAL);

    // set gyroscope config
    bno.err = bno055_set_gyr_conf(as_ptr(bno), BNO_GYR_RANGE_2000_DPS,
                                  BNO_GYR_BAND_32, BNO_GYR_MODE_NORMAL);

    // set BNO units
    bno.err = bno055_set_unit(as_ptr(bno), BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,
                              BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
    if (bno.err != BNO_OK) {
        usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
    } else {
        usart_printf(port, "[BNO] units set!\n");
    }
    delay_ms(1000);
    // set BNO power mode
    bno.err = bno055_set_pwr_mode(as_ptr(bno), BNO_PWR_NORMAL);
    if (bno.err != BNO_OK) {
        usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
    } else {
        usart_printf(port, "[BNO] power mode set!\n");
    }
    usart_printf(port, "\n");

    // const clock_t *test = &RCC_25MHZ_TO_84MHZ;
    u8 bit_test = 0;

    // ========================| Datatype Testing |=========================
    vec3 v1 = v3(4, 2, 0);
    vec4 v5 = v4(1, 2, 3, 4);
    vec3 v3;

    v1 = vec_mult(v1, 3);
    str v1_str = new_str(40);
    vec_to_str(v5, v1_str);
    usart_printf(port, "Vector test %s\n", v1_str);
    if (type(cycle) == type_u32) {
        usart_printf(port, "[Test] Type test: %s, %s\n", type_str(cycle),
                     type_str(bit_test));
    }

    // Initialize Timer
    if ((err_tim = timer_init(&tim5)) != OK) {
        usart_printf(port, "[TIM5] error: %s\n", timer_err_str(err_tim));
    } else {
        usart_printf(port, "[TIM5] ok!\n");
    }
    usart_printf(port, "\n");

    usart_printf(port, "[System] Starting main loop...\n\n");
    delay_ms(500);
    i2c_write(i2c1, BNO_ADDR, BNO_TEMP_SOURCE, 0x00);

    pin_high(PA8);
    str gyro_str = new_str(40);
    byte opr_mode, pwr_mode;
    while (1) {
        // ============================| Read BNO sensor data
        // |==============================
        i2c_err = i2c_read(i2c1, BNO_ADDR, BNO_OPR_MODE, as_ptr(opr_mode));
        if (i2c_err != OK) {
            usart_printf(port, "[I2C] error: %s\n", i2c_get_err_str(i2c_err));
        }
        i2c_err = i2c_read(i2c1, BNO_ADDR, BNO_PWR_MODE, as_ptr(pwr_mode));
        if (i2c_err != OK) {
            usart_printf(port, "[I2C] error: %s\n", i2c_get_err_str(i2c_err));
        }
        bno.err = bno055_gyro(as_ptr(bno), as_ptr(gyro));
        if (bno.err != BNO_OK) {
            usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
        }
        bno.err = bno055_acc(as_ptr(bno), as_ptr(acc));
        if (bno.err != BNO_OK) {
            usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
        }
        bno.err = bno055_temperature(as_ptr(bno), as_ptr(temperature));
        if (bno.err != BNO_OK) {
            usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
        }
        bno.err = bno055_mag(as_ptr(bno), as_ptr(mag));
        if (bno.err != BNO_OK) {
            usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
        }
        bno.err = bno055_euler(as_ptr(bno), as_ptr(euler));
        if (bno.err != BNO_OK) {
            usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
        }
        bno.err = bno055_linear_acc(as_ptr(bno), as_ptr(lia));
        if (bno.err != BNO_OK) {
            usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
        }
        bno.err = bno055_gravity(as_ptr(bno), as_ptr(gravity));
        if (bno.err != BNO_OK) {
            usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
        }
        bno.err = bno055_quaternion(as_ptr(bno), as_ptr(quat));
        if (bno.err != BNO_OK) {
            usart_printf(port, "[BNO] error: %s\n", bno055_err_str(bno.err));
        }

        vec_to_str(gyro, gyro_str);

        // ============================| Print sensor data
        // |==============================
        usart_printf(
            port,
            "time: %02ldh%02dm%02ds -->  temperature: %02d*C  "
            // "opr mode: %02d => pwr mode: %02d => "
            "gyro x: % 08.1f  y: % 08.1f  z: % 08.1f => "
            "acc x: % 08.1f  y: %08.1f  z: %08.1f => "
            // "linear acc x: % 08.1f  y: %08.1f  z: %08.1f => "
            // "gravity x: % 08.1f  y: %08.1f  z: %08.1f => "
            // "quaternions w: % 08.4f  x: % 08.4f  y: %08.4f  z: %08.4f => "
            "euler roll: % 08.1f  pitch: %08.1f  yaw: %08.1f\r",
            // "mag x: % 08.1f  y: %08.1f  z: %08.1f\r",
            hour, min, cycle, temperature,
            // opr_mode, pwr_mode, gyro.x,
            gyro.y, gyro.z, acc.x, acc.y, acc.z,
            // lia.x,
            // lia.y,
            // lia.z,
            // gravity.x,
            // gravity.y,
            // gravity.z,
            // quat.w,
            // quat.x,
            // quat.y,
            // quat.z,
            euler.x, euler.y, euler.z
            // mag.x,
            // mag.y,
            // mag.z,
        );

        if (cycle == 60) {
            inc(min, 1);
            cycle = 0;
        }
        if (min == 60) {
            inc(hour, 1);
            min = 0;
        }
        delay_ms(50);
        pin_low(PA8);
    }
}
