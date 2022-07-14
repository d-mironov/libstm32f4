#include "bno.h"
#include <string.h>
#include "../../gpio.h"
#include "../../delay.h"
#include "../../i2c.h"
#include "../../cosmic.h"
#include "../../error.h"

bool bno055_init(bno055 *bno) {
    u8 id;
    error err;
    error_bno err_bno;
    err = i2c_read(deref(bno->i2c), BNO_ADDR, BNO_CHIP_ID, as_ptr(id));
    if (err != OK) {
        _i2c_send_stop(deref(bno->i2c));
        return false;
    }
    if (id != BNO_DEF_CHIP_ID) {
        delayMs(1000);
        if (i2c_read(deref(bno->i2c), BNO_ADDR, BNO_CHIP_ID, as_ptr(id)) != OK) {
            _i2c_send_stop(deref(bno->i2c)); 
            return false;
        }
        if (id != BNO_DEF_CHIP_ID) {
            _i2c_send_stop(deref(bno->i2c));
            return false;
        }
        _i2c_send_stop(deref(bno->i2c)); 
        return false;
    } 
    // set operation mode to config mode
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return false;
    }
    delayMs(2);
    //bno055_reset(bno);
    //delayMs(1000);
    
    if (bno055_set_pwr_mode(bno, BNO_PWR_NORMAL) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return false;
    }
    delayMs(10);

    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return false;
    }
    delayMs(BNO_CONFIG_TIME_DELAY+5);

    //bno055_on(bno);
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return false;
    }
    delayMs(BNO_ANY_TIME_DELAY+5); 
    return true;
}



error_bno bno055_set_pwr_mode(bno055 *bno, const bno_pwr_t pwr) {
    if (bno == NULL) {
        bno->err = BNO_ERR_NULL_PTR;
        return BNO_ERR_NULL_PTR;
    }

    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        bno->err = BNO_ERR_I2C;
        return BNO_ERR_I2C;
    }

    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        bno->err = BNO_ERR_I2C;
        return BNO_ERR_I2C;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_PWR_MODE, pwr) != OK) {
        bno->err = BNO_ERR_I2C;
        return BNO_ERR_I2C;
    }
    bno->_pwr_mode = pwr;
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        bno->err = BNO_ERR_SETTING_PAGE;
        return BNO_ERR_SETTING_PAGE;
    }

    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        bno->err = BNO_ERR_I2C;
        return BNO_ERR_I2C;
    }


    delayMs(2);
    bno->err = BNO_OK;
    return BNO_OK;
}

error_bno bno055_set_page(bno055 *bno, const bno_page_t page) {
    if (page > 0x01) {
        return BNO_ERR_PAGE_TOO_HIGH;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_PAGE_ID, page) != OK) {
        return BNO_ERR_I2C;
    }
    bno->_page = page;
    delayMs(2);
    return BNO_OK;
}

error_bno bno055_set_opmode(bno055 *bno, const bno_opmode_t mode) {
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_OPR_MODE, mode) != OK) {
        return BNO_ERR_I2C;
    }
    delayMs(BNO_ANY_TIME_DELAY+5);
    return BNO_OK;
}

error_bno bno055_set_acc_conf(bno055 *bno,
                           const bno_acc_range_t range,
                           const bno_acc_band_t bandwidth,
                           const bno_acc_mode_t mode) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_1) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_ACC_CONFIG, range|bandwidth|mode) != OK) {
        return BNO_ERR_I2C;
    }
    bno->_acc_mode = mode;
    bno->_acc_bandwidth = bandwidth;
    bno->_acc_range = range;
    bno055_set_page(bno, BNO_PAGE_0);
    
    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    delayMs(2);
    return BNO_OK;
}

error_bno bno055_set_mag_conf(bno055 *bno,
                           const bno_mag_rate_t out_rate,
                           const bno_mag_pwr_t pwr_mode,
                           const bno_mag_mode_t mode) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_1) != BNO_OK) {
        return BNO_ERR_I2C;
    } if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_MAG_CONFIG, out_rate|pwr_mode|mode) != OK) {
        return BNO_ERR_I2C;
    }
    bno->_mag_mode = mode;
    bno->_mag_out_rate = out_rate;
    bno->_mag_pwr_mode = pwr_mode;

    // Set Operation mode to selected mode
    bno055_set_page(bno, BNO_PAGE_0);
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_SETTING_PAGE;
    }
    delayMs(2);
    return BNO_OK;
}

error_bno bno055_set_gyr_conf(bno055 *bno,
                           const bno_gyr_range_t range,
                           const bno_gyr_band_t bandwidth,
                           const bno_gyr_mode_t mode) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_1) != BNO_OK) {
        return BNO_ERR_SETTING_PAGE;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_GYR_CONFIG_0, range|bandwidth) != OK) {
        return BNO_ERR_I2C;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_GYR_CONFIG_1, mode) != OK) {
        return BNO_ERR_I2C;
    }
    bno->_gyr_mode = mode;
    bno->_gyr_bandwidth = bandwidth;
    bno->_gyr_range = range;
    bno055_set_page(bno, BNO_PAGE_0);
    
    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }

    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_SETTING_PAGE;
    }
    delayMs(2);
    return BNO_OK;
}


/**
 * Set units for BNO sensor
 * @param bno BNO structure
 * @param [t_unit](BNO_TEMP_UNIT_C,BNO_TEMP_UNIT_F) Temperature unit
 * @param [g_unit](BNO_GYR_UNIT_DPS,BNO_GYR_UNIT_RPS) Gyroscope unit
 * @param [a_unit](BNO_ACC_UNITSEL_M_S2,BNO_ACC_UNITSEL_M_S2) Accelerometer unit
 * @param [e_unit](BNO_EUL_UNIT_DEG,BNO_EUL_UNIT_RAD) Euler angles unit
 *
 * @return error_bno error code
 */
error_bno bno055_set_unit(bno055 *bno,
                       const bno_temp_unitsel_t t_unit,
                       const bno_gyr_unitsel_t g_unit,
                       const bno_acc_unitsel_t a_unit,
                       const bno_eul_unitsel_t e_unit) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_UNIT_SEL, t_unit|g_unit|a_unit|e_unit) != OK) {
        return BNO_ERR_I2C;
    }
    bno->_gyr_unit = g_unit;
    bno->_acc_unit = a_unit;
    bno->_eul_unit = e_unit;
    bno->_temp_unit = t_unit;

    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    delayMs(2);
    return BNO_OK;
}

error_bno bno055_set_temp_src(bno055 *bno, const enum bno_temp_src src) {
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_TEMP_SOURCE, src) != OK) {
        return BNO_ERR_I2C;
    }

    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    delayMs(2);
    return BNO_OK;
}

error_bno bno055_reset(bno055 *bno) {
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_SYS_TRIGGER, 0x20) != OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

error_bno bno055_on(bno055 *bno) {
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_SYS_TRIGGER, 0x0) != OK) {
        return BNO_ERR_I2C;
    }
    return BNO_OK;
}

error_bno bno055_temperature(bno055 *bno, i8 *temp) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c; 
    u8 data;
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read(deref(bno->i2c), BNO_ADDR, BNO_TEMP, &data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *temp = (bno->_temp_unit) ? data*2 : data;
    return BNO_OK;
}


/**
 * BNO055 Euler roll data
 * @brief Euler roll 
 * Get the euler roll data
 *
 * @param bno : BNO055 struct
 * @param _roll  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_euler_roll(bno055 *bno, f32 *roll) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_EUL_ROLL_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    deref(roll) = ((i16)(data[0] | (data[1] << 8))) /
        ((bno->_eul_unit == BNO_EUL_UNIT_DEG) ? 16.0f : 900.0f); 
    return BNO_OK;
}

/**
 * BNO055 Euler pitch data
 * @brief Euler pitch 
 * Get the euler pitch data
 *
 * @param bno : BNO055 struct
 * @param _pitch  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_euler_pitch(bno055 *bno, f32 *pitch) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_EUL_PITCH_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    deref(pitch) = ((i16)(data[0] | (data[1] << 8))) /
        ((bno->_eul_unit == BNO_EUL_UNIT_DEG) ? 16.0f : 900.0f); 
    return BNO_OK;
}

/**
 * BNO055 Euler yaw data
 * @brief Euler yaw 
 * Get the euler yaw data
 *
 * @param bno : BNO055 struct
 * @param _yaw  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_euler_yaw(bno055 *bno, f32 *yaw) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_EUL_HEADING_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    deref(yaw) = ((i16)(data[0] | (data[1] << 8))) /
        ((bno->_eul_unit == BNO_EUL_UNIT_DEG) ? 16.0f : 900.0f); 
    return BNO_OK;
}

/**
 * BNO055 all euler angles
 * @brief Euler angles
 * Get the euler angles depending 
 * on selected output unit.
 * => default: `deg`
 *
 * @param bno : BNO055 struct
 * @param vec  : `vec3` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_euler(bno055 *bno, vec3 *vec) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[6];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_EUL_ROLL_LSB, 6, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    deref(vec).x = ((i16)(data[0] | (data[1] << 8))) / 
        ((bno->_eul_unit == BNO_EUL_UNIT_DEG) ? 16.0f : 900.0f); 
    deref(vec).y = ((i16)(data[2] | (data[3] << 8))) / 
        ((bno->_eul_unit == BNO_EUL_UNIT_DEG) ? 16.0f : 900.0f); 
    deref(vec).z = ((i16)(data[4] | (data[5] << 8))) / 
        ((bno->_eul_unit == BNO_EUL_UNIT_DEG) ? 16.0f : 900.0f); 
    return BNO_OK;
}

/**
 * BNO055 Gyroscope X-axis data
 * @brief Gyroscope X-axis
 * Get the data on the X-axis of the Gyroscope
 *
 * @param bno : BNO055 struct
 * @param _x  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_gyro_x(bno055 *bno, f32 *_x) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GYR_DATA_X_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_x = ((i16) data[0] | (((i16)data[1]) << 8)) /
        ((bno->_gyr_unit == BNO_GYR_UNIT_DPS) ? 16.0f : 900.0f); 
    return BNO_OK;
}

/**
 * BNO055 Gyroscope Y-axis data
 * @brief Gyroscope Y-axis
 * Get the data on the Y-axis of the Gyroscope
 *
 * @param bno : BNO055 struct
 * @param _y  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_gyro_y(bno055 *bno, f32 *_y) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GYR_DATA_Y_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_y = ((i16) data[0] | (((i16)data[1]) << 8)) / 
        ((bno->_gyr_unit == BNO_GYR_UNIT_DPS) ? 16.0f : 900.0f); 
    return BNO_OK;
}

/**
 * BNO055 Gyroscope Z-axis data
 * @brief Gyroscope Z-axis
 * Get the data on the Z-axis of the Gyroscope
 *
 * @param bno : BNO055 struct
 * @param _z  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_gyro_z(bno055 *bno, f32 *_z) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GYR_DATA_Z_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_z = ((i16) data[0] | (((i16)data[1]) << 8)) / 
        ((bno->_gyr_unit == BNO_GYR_UNIT_DPS) ? 16.0f : 900.0f); 
    return BNO_OK;
}


/**
 * BNO055 Gyroscope all data
 * @brief Gyroscope data
 * Get the data of the Gyroscope depending 
 * on selected output unit.
 * => default: `dps`
 *
 * @param bno : BNO055 struct
 * @param vec  : `vec3` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_gyro(bno055 *bno, vec3 *vec) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[6];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GYR_DATA_X_LSB, 6, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }

    deref(vec).x = as(f32, ((i16) ((data[0]) | ((data[1]) << 8)))) /
        ((bno->_gyr_unit == BNO_GYR_UNIT_DPS) ? 16.0f : 900.0f); 
    deref(vec).y = as(f32, ((i16) ((data[2]) | ((data[3]) << 8)))) /
        ((bno->_gyr_unit == BNO_GYR_UNIT_DPS) ? 16.0f : 900.0f); 
    deref(vec).z = as(f32, ((i16) ((data[4]) | ((data[5]) << 8)))) /
        ((bno->_gyr_unit == BNO_GYR_UNIT_DPS) ? 16.0f : 900.0f); 
    return BNO_OK;
}


/**
 * BNO055 Accelerometer X-axis data
 * @brief Accelerometer X-axis
 * Get the data on the X-axis of the Accelerometer
 *
 * @param bno : BNO055 struct
 * @param _x  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_acc_x(bno055 *bno, f32 *_x) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_ACC_DATA_X_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_x = ((i16) data[0] | (((i16)data[1]) << 8)) / 
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}

/**
 * BNO055 Accelerometer Y-axis data
 * @brief Accelerometer Y-axis
 * Get the data on the Y-axis of the Accelerometer
 *
 * @param bno : BNO055 struct
 * @param _y  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_acc_y(bno055 *bno, f32 *_y) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_ACC_DATA_Y_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_y = ((i16) data[0] | (((i16)data[1]) << 8)) /  
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}

/**
 * BNO055 Accelerometer Z-axis data
 * @brief Accelerometer Z-axis
 * Get the data on the Z-axis of the Accelerometer
 *
 * @param bno : BNO055 struct
 * @param _z  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_acc_z(bno055 *bno, f32 *_z) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_ACC_DATA_Z_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_z = ((i16) data[0] | (((i16)data[1]) << 8)) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}


/**
 * BNO055 Accelerometer all data
 * @brief Accelerometer data
 * Get the data of the Accelerometer depending 
 * on selected output unit.
 * => default: `m/s^2`
 *
 * @param bno : BNO055 struct
 * @param vec  : `vec3` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_acc(bno055 *bno, vec3 *vec) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[6];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_ACC_DATA_X_LSB, 6, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }

    deref(vec).x = as(f32, ((i16) ((data[0]) | ((data[1]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    deref(vec).y = as(f32, ((i16) ((data[2]) | ((data[3]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    deref(vec).z = as(f32, ((i16) ((data[4]) | ((data[5]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}


/**
 * BNO055 magnetometer X-axis data
 * @brief magnetometer X-axis
 * Get the data on the X-axis of the magnetometer 
 *
 * @param bno : BNO055 struct
 * @param _x  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_mag_x(bno055 *bno, f32 *_x) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_MAG_DATA_X_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_x = ((i16) data[0] | (((i16)data[1]) << 8)) / 16.0f;
    return BNO_OK;
}

/**
 * BNO055 magnetometer Y-axis data
 * @brief magnetometer Y-axis
 * Get the data on the Y-axis of the magnetometer 
 *
 * @param bno : BNO055 struct
 * @param _y  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_mag_y(bno055 *bno, f32 *_y) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_MAG_DATA_Y_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_y = ((i16) data[0] | (((i16)data[1]) << 8)) / 16.0f; 
    return BNO_OK;
}

/**
 * BNO055 magnetometer Z-axis data
 * @brief magnetometer Z-axis
 * Get the data on the Z-axis of the magnetometer 
 *
 * @param bno : BNO055 struct
 * @param _z  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_mag_z(bno055 *bno, f32 *_z) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_MAG_DATA_Z_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_z = ((i16) data[0] | (((i16)data[1]) << 8)) / 16.0f; 
    return BNO_OK;
}


/**
 * BNO055 Magnetometer all data
 * @brief Magnetometer data
 * Get the data of the Accelerometer depending 
 * on selected output unit.
 * => default: `m/s^2`
 *
 * @param bno : BNO055 struct
 * @param vec  : `vec3` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_mag(bno055 *bno, vec3 *vec) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[6];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_MAG_DATA_X_LSB, 6, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }

    deref(vec).x = as(f32, ((i16) ((data[0]) | ((data[1]) << 8)))) / 16.0f;
    deref(vec).y = as(f32, ((i16) ((data[2]) | ((data[3]) << 8)))) / 16.0f;
    deref(vec).z = as(f32, ((i16) ((data[4]) | ((data[5]) << 8)))) / 16.0f;
    return BNO_OK;
}


/**
 * BNO055 Linear Acceleration X-axis data
 * @brief Linear Acceleration X-axis
 * Get the data on the X-axis of the Linear Acceleration
 *
 * @param bno : BNO055 struct
 * @param _x  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_linear_acc_x(bno055 *bno, f32 *_x) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_LIA_DATA_X_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_x = ((i16) data[0] | (((i16)data[1]) << 8)) / 
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}

/**
 * BNO055 Linear Acceleration Y-axis data
 * @brief Linear Acceleration Y-axis
 * Get the data on the Y-axis of the Linear Acceleration
 *
 * @param bno : BNO055 struct
 * @param _y  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_linear_acc_y(bno055 *bno, f32 *_y) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_LIA_DATA_Y_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_y = ((i16) data[0] | (((i16)data[1]) << 8)) /  
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}

/**
 * BNO055 Linear Acceleration Y-axis data
 * @brief Linear Acceleration Y-axis
 * Get the data on the Y-axis of the Linear Acceleration
 *
 * @param bno : BNO055 struct
 * @param _y  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_linear_acc_z(bno055 *bno, f32 *_z) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_LIA_DATA_Z_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_z = ((i16) data[0] | (((i16)data[1]) << 8)) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}


/**
 * BNO055 all Linear Acceleration
 * @brief Linear acceleration data
 * Get the linear acceleration data depending
 * on selected output unit.
 * => default: `m/s^2`
 *
 * @param bno : BNO055 struct
 * @param vec  : `vec3` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_linear_acc(bno055 *bno, vec3 *vec) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[6];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_LIA_DATA_X_LSB, 6, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }

    deref(vec).x = as(f32, ((i16) ((data[0]) | ((data[1]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    deref(vec).y = as(f32, ((i16) ((data[2]) | ((data[3]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    deref(vec).z = as(f32, ((i16) ((data[4]) | ((data[5]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}


/**
 * BNO055 Gravite X-axis data
 * @brief Gravity X-axis
 * Get the Gravity data on the X-axis
 *
 * @param bno : BNO055 struct
 * @param _x  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_gravity_x(bno055 *bno, f32 *_x) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GRV_DATA_X_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_x = ((i16) data[0] | (((i16)data[1]) << 8)) / 
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}

/**
 * BNO055 Gravity Y-axis data
 * @brief Gravity Y-axis
 * Get the Gravity data on the Y-axis
 *
 * @param bno : BNO055 struct
 * @param _y  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_gravity_y(bno055 *bno, f32 *_y) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GRV_DATA_Y_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_y = ((i16) data[0] | (((i16)data[1]) << 8)) /  
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}

/**
 * BNO055 Gravity Z-axis data
 * @brief Gravity Z-axis
 * Get the Gravity data on the Z-axis
 *
 * @param bno : BNO055 struct
 * @param _z  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_gravity_z(bno055 *bno, f32 *_z) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GRV_DATA_Z_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_z = ((i16) data[0] | (((i16)data[1]) << 8)) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}


/**
 * BNO055 all Gravity data
 * @brief Gravity data
 * Get the gravity data depending
 * on selected output unit.
 * => default: `m/s^2`
 *
 * @param bno : BNO055 struct
 * @param vec  : `vec3` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_gravity(bno055 *bno, vec3 *vec) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[6];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GRV_DATA_X_LSB, 6, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }

    deref(vec).x = as(f32, ((i16) ((data[0]) | ((data[1]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    deref(vec).y = as(f32, ((i16) ((data[2]) | ((data[3]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    deref(vec).z = as(f32, ((i16) ((data[4]) | ((data[5]) << 8)))) /
        ((bno->_acc_unit == BNO_ACC_UNITSEL_M_S2) ? 100.0f : 1.0f); 
    return BNO_OK;
}

/**
 * BNO055 Quaternion W-heading
 * @brief Quaternion W-heading
 * Get the Quaternion data on W-heading
 *
 * @param bno : BNO055 struct
 * @param _w  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_quaternion_w(bno055 *bno, f32 *_w) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_QUA_DATA_W_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_w = ((i16) data[0] | (((i16)data[1]) << 8)) / BNO_QUA_POW_2_14;
    return BNO_OK;
}
/**
 * BNO055 Quaternion X-axis data
 * @brief Quaternion X-axis
 * Get the Quaternion data on the X-axis
 *
 * @param bno : BNO055 struct
 * @param _x  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_quaternion_x(bno055 *bno, f32 *_x) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_QUA_DATA_X_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_x = ((i16) data[0] | (((i16)data[1]) << 8)) / BNO_QUA_POW_2_14;
    return BNO_OK;
}

/**
 * BNO055 Quaternion Y-axis data
 * @brief Quaternion Y-axis
 * Get the Quaternion data on the Y-axis
 *
 * @param bno : BNO055 struct
 * @param _y  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_quaternion_y(bno055 *bno, f32 *_y) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_QUA_DATA_Y_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_y = ((i16) data[0] | (((i16)data[1]) << 8)) / BNO_QUA_POW_2_14;
    return BNO_OK;
}

/**
 * BNO055 Quaternion Z-axis data
 * @brief Quaternion Z-axis
 * Get the Quaternion data on the Z-axis
 *
 * @param bno : BNO055 struct
 * @param _z  : `f32` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_quaternion_z(bno055 *bno, f32 *_z) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[2];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_QUA_DATA_Z_LSB, 2, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    *_z = ((i16) data[0] | (((i16)data[1]) << 8)) / BNO_QUA_POW_2_14;
    return BNO_OK;
}


/**
 * BNO055 all Gravity data
 * @brief Gravity data
 * Get the gravity data depending
 * on selected output unit.
 * => default: `m/s^2`
 *
 * @param bno : BNO055 struct
 * @param vec  : `vec3` buffer
 *
 * @return error code
 * @return0 `BNO_ERR_I2C` on error
 * @return1 `BNO_OK` on success
 */
error_bno bno055_quaternion(bno055 *bno, vec4 *vec) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data[8];
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return err_bno;
        }
    }
    err_i2c = i2c_read_burst(deref(bno->i2c), BNO_ADDR, BNO_GRV_DATA_X_LSB, 6, data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    deref(vec).w = as(f32, ((i16) ((data[0]) | ((data[1]) << 8)))) / BNO_QUA_POW_2_14;
    deref(vec).x = as(f32, ((i16) ((data[2]) | ((data[3]) << 8)))) / BNO_QUA_POW_2_14;
    deref(vec).y = as(f32, ((i16) ((data[4]) | ((data[5]) << 8)))) / BNO_QUA_POW_2_14;
    deref(vec).z = as(f32, ((i16) ((data[6]) | ((data[7]) << 8)))) / BNO_QUA_POW_2_14;
    return BNO_OK;
}


bool bno055_calibrate(bno055 *bno) {
    
    return true;
}

error_bno bno055_axis_remap(bno055 *bno, 
                            const bno_axis_remap xrmap,
                            const bno_axis_remap yrmap,
                            const bno_axis_remap zrmap) {
    if (xrmap > BNO_AXIS_REMAP_Z || yrmap > BNO_AXIS_REMAP_Z || zrmap > BNO_AXIS_REMAP_Z) {
        return BNO_ERR_AXIS_REMAP;
    }
    
    if (bno055_set_opmode(bno, BNO_MODE_CONFIG) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    if (bno055_set_page(bno, BNO_PAGE_0) != BNO_OK) {
        return BNO_ERR_I2C;
    }
    if (i2c_write(deref(bno->i2c), BNO_ADDR, BNO_AXIS_MAP_CONFIG, xrmap|yrmap|zrmap) != OK) {
        return BNO_ERR_I2C;
    }
    // Set Operation mode to selected mode
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
    delayMs(2);


    return  BNO_OK;
}


str bno055_err_str(const error_bno err) {
    switch (err) {
        case BNO_ERR_I2C:
            return "I2C error";
        case BNO_ERR_PAGE_TOO_HIGH:
            return "BNO wrong page -> must be 0 or 1";
        case BNO_ERR_SETTING_PAGE:
            return "";
        case BNO_ERR_NULL_PTR:
            return "BNO struct is null pointer";
        default:
            return "BNO ok";
    }
}

error_bno bno055_post_result(bno055 *bno, str buf) {
#ifdef BNO_SAFE_OPMODE
    if (bno055_set_opmode(bno, bno->mode) != BNO_OK) {
        _i2c_send_stop(deref(bno->i2c));
        return BNO_ERR_I2C;
    }
#endif
    error_bno err_bno;
    error err_i2c;
    u8 data;
    if (bno->_page != BNO_PAGE_0) {
        if ((err_bno = bno055_set_page(bno, BNO_PAGE_0)) != BNO_OK) {
            return BNO_ERR_I2C;
        }
    }
    err_i2c = i2c_read(deref(bno->i2c), BNO_ADDR, BNO_ST_RESULT, &data);
    if (err_i2c != OK) {
        return BNO_ERR_I2C;
    }
    strcat(buf, (bit_is_set(data,0)) ? "ACC...ok\n" : "ACC...failed\n");
    strcat(buf, (bit_is_set(data,1)) ? "MAG...ok\n" : "MAG...failed\n");
    strcat(buf, (bit_is_set(data,2)) ? "GYR...ok\n" : "GYR...failed");
    strcat(buf, "\0");
    return BNO_OK;
}
