/*
 * TwoWire interface for STM32F4xx boards
 *
 * This will support both SMBus and I2C
 *
 *
 * changelog:
 * |_____v0.1
 * |       |___ added init function (tested)
 * |       |___ added byte read function (tested)
 * |       |___ added burst byte read function
 * |       |___ added burst write function
 * |
 *
 * @author: cosmicraccoon (aka Daniel Mironow)
 * @version: v0.1
 *
 */
#ifndef _I2C_H
#define _I2C_H

#include <stm32f4xx.h>
#include <stdbool.h>
#include "bitutils.h"
#include "cosmic.h"

#define I2C_STD_MODE    0x00
#define I2C_FAST_MODE   0x01

#define I2C_FREQ_MIN_SM     0x02
#define I2C_FREQ_MIN_FM     0x04
#define I2C_FREQ_MAX        0x32

//  Standard mode timing in <s>
#define I2C_SM_SCLL         0.0000047
#define I2C_SM_SCLH         0.000004
#define I2C_SM_SCL_RISE_MAX 0.000001
#define I2C_SM_SCL_FALL_MAX 0.0000003

//  Fast mode timing in <s>
#define I2C_FM_SCLL         0.0000013
#define I2C_FM_SCLH         0.0000006
#define I2C_FM_SCL_RISE_MAX 0.0000003
#define I2C_FM_SCL_FALL_MAX 0.0000003

#define I2C_DUTY_2      0x00
#define I2C_DUTY_16_9   0x01

#define I2C_READY       0x00
#define I2C_BUSY_RX     0x01
#define I2C_BUSY_TX     0x02

#define I2C_BERR_OFFSET     (u8)(0x08)
#define I2C_ARLO_OFFSET     (u8)(0x09)
#define I2C_AF_OFFSET       (u8)(0x0A)
#define I2C_OVR_OFFSET      (u8)(0x0B)
#define I2C_PECERR_OFFSET   (u8)(0x0C)

#define I2C_BERR        (1 << I2C_BERR_OFFSET)
#define I2C_ARLOERR     (1 << I2C_ARLO_OFFSET)
#define I2C_AFERR       (1 << I2C_AF_OFFSET)
#define I2C_OVRERR      (1 << I2C_OVR_OFFSET)
#define I2C_PECERR      (1 << I2C_PECERR_OFFSET)



#define I2C_ITBUFEN_BIT     (0x0A)
#define I2C_ITEVTEN_BIT     (0x09)
#define I2C_ITERREN_BIT     (0x08)


#define I2C_READ            true
#define I2C_WRITE           false


// typedef enum _i2c_err_h {
//     I2C_OK,
//     I2C_ERR_PORT_NOT_AVAILABLE,
//     I2C_ERR_FREQ_TOO_LOW,
//     I2C_ERR_FREQ_TOO_HIGH,
//     I2C_ERR_BUS,
//     I2C_ERR_ARBLOSS,
//     I2C_ERR_AF,
//     I2C_ERR_OVR,
//     I2C_ERR_PEC,
//     I2C_ERR_TIMEOUT,
//     I2C_ERR_SMBALERT,
//     I2C_ERR_PORT_UNDEFINED,
//     I2C_ERR_NOT_CONFIGURED,
//     I2C_ERR_BUS_TIMEOUT,
// } i2c_err_t;

typedef struct __twowire_it_handle {
    u8 *tx_buf;
    u8 *rx_buf;
    u8 status;
} __twowire_it_handle_t;

typedef struct _I2C {
    I2C_TypeDef *i2c;
    u8 frequency;
    u8 mode;
    u8 duty;
    u8 timeout;
    bool _set_up;
    bool interrupt_driven;
    bool slave;
} i2c;


//  init
error       i2c_init            (i2c *port);
// read
error       i2c_read            (i2c port, u8 slave, u8 memaddr, u8 *data);
error       i2c_read_burst      (i2c port, u8 slave, u8 memaddr, u8 n, u8 *data);
// write
error       i2c_write           (i2c port, u8 slave, u8 memaddr, u8 data);
error       i2c_write_burst     (i2c port, u8 slave, u8 memaddr, u8 n, u8 *data);
// errors
error       i2c_get_err         (i2c port);
str         i2c_get_err_str     (error err);
error       i2c_handle_err      (i2c port, error err);

f32         _i2c_ccr_calc       (i2c *port);
f32         _i2c_trise_calc     (i2c *port);

error       _i2c_send_start     (i2c port);
error       _i2c_send_addr      (i2c port, u8 addr, bool rw);
error       _i2c_send_data      (i2c port, u8 data);
error       _i2c_send_stop      (i2c port);



#endif
