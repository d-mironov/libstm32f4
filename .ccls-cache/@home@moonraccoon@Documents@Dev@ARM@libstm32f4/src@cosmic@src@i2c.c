#include "../i2c.h"
#include "../gpio.h"
#include <stm32f4xx.h>
#include <stdbool.h>
#include "../delay.h"
#include "../cosmic.h"


/**
 * I2C initialization:
 * Resets the peripheral and sets the new settings
 *
 * @param port I2C port configuration struct
 *
 * @return error error code
 */
error i2c_init(i2c *port) {
    // FIXME: port stuck when at least 1 device not powered on
    port->_set_up = false;
    if (port->i2c == I2C1) {
        // enabling GPIO 6,7, selecting alternate function, setting speed
        pin_enable(PB6, GPIO_ALTERNATE);
        pin_enable(PB7, GPIO_ALTERNATE);
        pin_settings(PB6, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        pin_settings(PB7, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        pin_select_alternate(PB6, GPIO_AF04);
        pin_select_alternate(PB7, GPIO_AF04);
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    } else if (port->i2c == I2C2) {
        pin_enable(PB10, GPIO_ALTERNATE);
        pin_enable(PB11, GPIO_ALTERNATE);
        pin_settings(PB10, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        pin_settings(PB11, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        pin_select_alternate(PB10, GPIO_AF04);
        pin_select_alternate(PB11, GPIO_AF04);
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    } else if (port->i2c == I2C3) {
        pin_enable(PB8, GPIO_ALTERNATE);
        pin_enable(PC9, GPIO_ALTERNATE);
        pin_settings(PA8, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        pin_settings(PC9, GPIO_HIGH_SPEED, GPIO_PULL_UP, GPIO_OUT_OD);
        pin_select_alternate(PA8, GPIO_AF04);
        pin_select_alternate(PC9, GPIO_AF04);
        RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
    } else {
        return I2C_ERR_PORT_UNDEFINED;
    }

    if (port->mode == I2C_FAST_MODE && port->frequency < I2C_FREQ_MIN_FM ) {
        return I2C_ERR_FREQ_TOO_LOW;
    } else if (port->frequency < I2C_FREQ_MIN_SM) {
        return I2C_ERR_FREQ_TOO_LOW;
    } else if (port->frequency > I2C_FREQ_MAX) {
        return I2C_ERR_FREQ_TOO_HIGH;
    }

    if (port->timeout == 0) {
        port->timeout = 100;
    }

    // Reset CRx, CCR and TRISE
    (port->i2c)->CR1 = 0x0000;
    (port->i2c)->CR2 = 0x0000;
    (port->i2c)->CCR = 0x0000;
    (port->i2c)->TRISE = 0x0002;

    (port->i2c)->CR1 |= I2C_CR1_SWRST;
    (port->i2c)->CR1 &= ~I2C_CR1_SWRST;
    if (port->interrupt_driven) {
        if (port->i2c == I2C1) {
            NVIC_EnableIRQ(I2C1_ER_IRQn);
            NVIC_EnableIRQ(I2C1_EV_IRQn);
        } else if (port->i2c == I2C2) {
            NVIC_EnableIRQ(I2C2_ER_IRQn);
            NVIC_EnableIRQ(I2C2_EV_IRQn);
        } else if (port->i2c == I2C3) {
            NVIC_EnableIRQ(I2C3_ER_IRQn);
            NVIC_EnableIRQ(I2C3_EV_IRQn);
        }
        setbit_var(port->i2c->CR2, I2C_ITBUFEN_BIT);
        setbit_var(port->i2c->CR2, I2C_ITEVTEN_BIT);
        setbit_var(port->i2c->CR2, I2C_ITERREN_BIT);
    }

    (port->i2c)->CR2 |= port->frequency;
    (port->i2c)->CCR = (u32) _i2c_ccr_calc(port);
    (port->i2c)->CCR |= port->duty << 14;
    (port->i2c)->CCR |= port->mode << 15;
    (port->i2c)->TRISE |= (u32) _i2c_trise_calc(port);

    (port->i2c)->CR1 |= I2C_CR1_PE;
    port->_set_up = true;
    return OK;
}

/**
 * I2C CCR register calculation
 * @param port I2C init struct
 * @return ccr register value
 */
f32 _i2c_ccr_calc(i2c *port) {
    f32 t_high = 0;
    i32 freq = port->frequency * 1000000;
    if (port->mode == I2C_STD_MODE) {
        t_high = I2C_SM_SCL_RISE_MAX + I2C_SM_SCLH;
    } else if (port->mode == I2C_FAST_MODE) {
        t_high = I2C_SM_SCL_RISE_MAX + I2C_FM_SCLH;
    }

    f32 tmp = 1.0 / freq;
    if (port->mode == I2C_STD_MODE) {
        return t_high / tmp;
    } else if (port->mode == I2C_FAST_MODE) {
        if (port->duty == I2C_DUTY_2) {
            return t_high / tmp;
        } else if (port->duty == I2C_DUTY_16_9) {
            return t_high / (tmp * 9);
        } else {
            return 0.0;
        }
    }
    return 0;
}

/**
 * I2C TRISE register calculation
 * @param port I2C init struct
 * @return ccr register value
 */
f32 _i2c_trise_calc(i2c *port) {
    f32 freq = 1.0 / (port->frequency * 1000000.0);
    if (port->mode == I2C_STD_MODE) {
        return (I2C_SM_SCL_RISE_MAX / freq) + 1.0;
    } else if (port->mode == I2C_FAST_MODE) {
        return (I2C_FM_SCL_RISE_MAX / freq) + 1.0;
    } else {
        return 0.0;
    }
}


/**
 * I2C byte read
 * Read 1 byte from a slave device with `slave` address on Memory
 * address `memaddr`
 *
 * @param port - I2C object
 * @param slave - 7/10-bit slave address of slave
 * @param memaddr - Memory address to read from
 *
 * @return byte read from the I2C device (u8)
 */
error i2c_read(i2c port, u8 slave, u8 memaddr, u8 *data) {

    if (!port._set_up) {
        return I2C_ERR_NOT_CONFIGURED;
    }
    io i32 tmp;
    u8 out;
    error err;
    // TODO: fix infinite loop on wrong device
    //GPIO_write(PA8, GPIO_ON);
    SysTick->LOAD = (apb1_freq/1000);
    SysTick->VAL = 0x00;
    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

    for(u32 i = 0; i < port.timeout && (port.i2c)->SR2 & I2C_SR2_BUSY; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0);
        if ((err = i2c_get_err(port)) != OK) {
            return err;
        }
        if (i == port.timeout - 1) {
            return I2C_ERR_BUS_TIMEOUT;
        }
    }
    SysTick->CTRL = 0x00;
    //while((port.i2c)->SR2 & I2C_SR2_BUSY) {
    //    if ((err = I2C_get_err(port)) != OK) {
    //        return err;
    //    }
    //}
    // Generate start condition
    if ((err = _i2c_send_start(port)) != OK) {
        return err;
    }

    // Send address of slave
    if ((err = _i2c_send_addr(port, slave, I2C_WRITE)) != OK) {
        return err;
    }

    tmp = (port.i2c)->SR2;
    //send memory address
    (port.i2c)->DR = memaddr;
    // check TXE flag
    while(!((port.i2c)->SR1 & I2C_SR1_TXE));
    //// GENERATE RESTART CONDITION
    if ((err = _i2c_send_start(port)) != OK) {
        return err;
    }
    //// put address in read mode
    if ((err = _i2c_send_addr(port, slave, I2C_READ)) != OK) {
        return err;
    }
    //disable ACK
    (port.i2c)->CR1 &= ~I2C_CR1_ACK;
    tmp = (port.i2c)->SR2;
    //generate stop
    (port.i2c)->CR1 |= I2C_CR1_STOP;
    // check for RXNE flag
    while(!((port.i2c)->SR1 & I2C_SR1_RXNE));
    //GPIO_write(PA8, GPIO_ON);
    *data = (port.i2c)->DR;
    return OK;
}

/**
 * I2C Burst read
 * Read `n` number of bytes from slave starting
 * on memory address `memaddr` and store them in
 * and array
 *
 * @param port - I2C struct
 * @param slave - Slave address
 * @param memaddr - start address of burst read
 * @param n - number of bytes to read
 *
 * @return error-code - error code
 */
error i2c_read_burst(i2c port, u8 slave, u8 memaddr, u8 n, u8 *data) {
    if (!port._set_up) {
        return I2C_ERR_NOT_CONFIGURED;
    }
    io i32 tmp;
    error err;
    io u8 reg = memaddr;
    //while((port.i2c)->SR2 & I2C_SR2_BUSY) {
    //    if ((err = I2C_get_err(port)) != OK) {
    //        return err;
    //    }
    //}

    // Timeout for bus busy
    SysTick->LOAD = (apb1_freq/1000);
    SysTick->VAL = 0x00;
    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
    for(u32 i = 0; i < port.timeout && (port.i2c)->SR2 & I2C_SR2_BUSY; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0);
        if ((err = i2c_get_err(port)) != OK) {
            return err;
        }
        if (i == port.timeout - 1) {
            return I2C_ERR_BUS_TIMEOUT;
        }
    }
    SysTick->CTRL = 0x00;

    // GENERATE START CONDITION
    if ((err = _i2c_send_start(port)) != OK) {
        return err;
    }
    // *NEW* SEND SLAVE ADDRESS
    if ((err = _i2c_send_addr(port, slave, I2C_WRITE)) != OK) {
        return err;
    }
    tmp = (port.i2c)->SR2;
    //send memory address
    (port.i2c)->DR = memaddr;
    // check TXE flag
    while(!((port.i2c)->SR1 & I2C_SR1_TXE)) {
        if ((err = i2c_get_err(port)) != OK) {
            return err;
        }
    }
    // GENERATE  RESTART CONDITION
    if ((err = _i2c_send_start(port)) != OK) {
        return err;
    }


    // SEND READ ADDR IN READ MODE
    if ((err = _i2c_send_addr(port, slave, I2C_READ)) != OK) {
        return err;
    }


    tmp = (port.i2c)->SR2;
    //disable ACK
    (port.i2c)->CR1 |= I2C_CR1_ACK;

    for (io i32 i = 0; i < n; i++) {
        if (i == n-1) {
            (port.i2c)->CR1 &= ~I2C_CR1_ACK;
            (port.i2c)->CR1 |= I2C_CR1_STOP;
            while(!((port.i2c)->SR1 & I2C_SR1_RXNE));
            data[i] = (port.i2c)->DR;
            break;
        } else {
            while(!((port.i2c)->SR1 & I2C_SR1_RXNE));
            data[i] = (port.i2c)->DR;
        }
    }
    //while (n > 0U) {
    //    // if one byte
    //    if (n == 1) {
    //        (port.i2c)->CR1 &= ~I2C_CR1_ACK;
    //        (port.i2c)->CR1 |= I2C_CR1_STOP;
    //        while(!((port.i2c)->SR1 & I2C_SR1_RXNE));
    //        *data++ = (port.i2c)->DR;
    //        break;
    //    } else {
    //        while(!((port.i2c)->SR1 & I2C_SR1_RXNE));
    //        *data++ = (port.i2c)->DR;
    //        n--;
    //    }
    //}
    // check for RXNE flag
    //while(!((port.i2c)->SR1 & I2C_SR1_RXNE));


    return OK;
}



error i2c_write_burst(i2c port, u8 slave, u8 memaddr, u8 n, u8 *data) {
    if (!port._set_up) {
        return I2C_ERR_NOT_CONFIGURED;
    }
    io i32 tmp;
    error err;


    // Timeout for bus busy
    SysTick->LOAD = (apb1_freq/1000);
    SysTick->VAL = 0x00;
    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
    for(u32 i = 0; i < port.timeout && (port.i2c)->SR2 & I2C_SR2_BUSY; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0);
        if ((err = i2c_get_err(port)) != OK) {
            return err;
        }
        if (i == port.timeout - 1) {
            return I2C_ERR_BUS_TIMEOUT;
        }
    }
    SysTick->CTRL = 0x00;


    //while((port.i2c)->SR2 & I2C_SR2_BUSY) {
    //    if ((err = I2C_get_err(port)) != OK) {
    //        return err;
    //    }
    //}
    if ((err = _i2c_send_start(port)) != OK) {
        return err;
    }


    // SEND WRITE ADDRESS
    if ((err = _i2c_send_addr(port, slave, I2C_WRITE)) != OK) {
        return err;
    }


    tmp = (port.i2c)->SR2;
    // check TXE flag
    while(!((port.i2c)->SR1 & I2C_SR1_TXE));
    //send memory address
    (port.i2c)->DR = memaddr;
    for (u32 i = 0; i < n; i++) {
        while(!((port.i2c)->SR1 & I2C_SR1_TXE));
        (port.i2c)->DR = data[i];
    }
    while(!((port.i2c)->SR1 & I2C_SR1_BTF));
    (port.i2c)->CR1 |= I2C_CR1_STOP;

    return OK;
}

error i2c_write(i2c port, u8 slave, u8 memaddr, u8 data) {
    if (!port._set_up) {
        return I2C_ERR_NOT_CONFIGURED;
    }
    io i32 tmp;
    error err;


    // Timeout for bus busy
    SysTick->LOAD = (apb1_freq/1000);
    SysTick->VAL = 0x00;
    SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;
    for(u32 i = 0; i < port.timeout && (port.i2c)->SR2 & I2C_SR2_BUSY; i++) {
        while((SysTick->CTRL & CTRL_COUNTFLAG) == 0);
        if ((err = i2c_get_err(port)) != OK) {
            return err;
        }
        if (i == port.timeout - 1) {
            return I2C_ERR_BUS_TIMEOUT;
        }
    }
    SysTick->CTRL = 0x00;


    while((port.i2c)->SR2 & I2C_SR2_BUSY) {
        if ((err = i2c_get_err(port)) != OK) {
            return err;
        }
    }
    if ((err = _i2c_send_start(port)) != OK) {
        return err;
    }


    // SEND WRITE ADDRESS
    if ((err = _i2c_send_addr(port, slave, I2C_WRITE)) != OK) {
        return err;
    }


    tmp = (port.i2c)->SR2;
    // check TXE flag
    while(!((port.i2c)->SR1 & I2C_SR1_TXE));
    //send memory address
    (port.i2c)->DR = memaddr;
    while(!((port.i2c)->SR1 & I2C_SR1_TXE));
    (port.i2c)->DR = data;
    while(!((port.i2c)->SR1 & I2C_SR1_BTF));
    (port.i2c)->CR1 |= I2C_CR1_STOP;

    delayMs(1);
    return OK;
}

error i2c_get_err(i2c port) {
    if (port.i2c->SR1 & I2C_BERR) {
        return I2C_ERR_BUS;
    } else if (port.i2c->SR1 & I2C_ARLOERR) {
        return I2C_ERR_ARBLOSS;
    } else if (port.i2c->SR1 & I2C_AFERR) {
        return I2C_ERR_AF;
    } else if (port.i2c->SR1 & I2C_OVRERR) {
        return I2C_ERR_OVR;
    } else if (port.i2c->SR1 & I2C_PECERR) {
        return I2C_ERR_PEC;
    } else {
        return OK;
    }
    return OK;
}


/**
 * I2C send start on port
 * @param port port to generate start
 * @return err `OK` on success, otherwise `I2C_ERR_x`
 */
error _i2c_send_start(i2c port) {
    // return `I2C_ERR_NOT_CONFIGURED` if port is not set up
    if (!port._set_up) {
        return I2C_ERR_NOT_CONFIGURED;
    }
    error err = OK;
    // set ACK bit and generate start condition
    port.i2c->CR1 |= I2C_CR1_ACK;
    port.i2c->CR1 |= I2C_CR1_START;
    // wait until SB bit is set unless error occurs
    while (!(port.i2c->SR1 & I2C_SR1_SB)) {
        if ((err = i2c_get_err(port)) != OK) {
            return err;
        }
    }
    return err;
}


error _i2c_send_addr(i2c port, u8 addr, bool rw) {
    _i2c_send_data(port, (rw)?((addr<<1)|1):(addr<<1));
    error err = OK;
    while(!((port.i2c)->SR1 & I2C_SR1_ADDR)) {
        if ((err = i2c_get_err(port)) == I2C_ERR_AF) {
            return err;
        }
    }
    return err;
}

error _i2c_send_data(i2c port, u8 data) {
    port.i2c->DR = data;
    return OK;
}

error _i2c_send_stop(i2c port) {
    (port.i2c)->CR1 |= I2C_CR1_STOP;
    return OK;
}

str i2c_get_err_str(error err) {
    switch (err) {
    case I2C_ERR_AF:
        return "[I2C] acknowledge failure";
    case I2C_ERR_BUS:
        return "[I2C] bus error";
    case I2C_ERR_OVR:
        return "[I2C] overrun error";
    case I2C_ERR_PEC:
        return "[I2C] PEC error";
    case I2C_ERR_ARBLOSS:
        return "[I2C] arbitration loss";
    case I2C_ERR_TIMEOUT:
        return "[SMBus] timeout";
    case I2C_ERR_SMBALERT:
        return "[SMBus] SMBus alert";
    case I2C_ERR_FREQ_TOO_LOW:
        return "[I2C] set frequency too low";
    case I2C_ERR_FREQ_TOO_HIGH:
        return "[I2C] set frequency too high";
    case I2C_ERR_NOT_CONFIGURED:
        return "[I2C] port not set up: call `i2c_init()`";
    case I2C_ERR_PORT_UNDEFINED:
        return "[I2C] port unavailable for this MCU or undefined";
    case I2C_ERR_PORT_NOT_AVAILABLE:
        return "[I2C] port not available";
    default:
        return "[I2C] Ok!";
    }
}


error i2c_handle_err(i2c port, error err) {
    return OK;
}

