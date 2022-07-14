# BNO055 IMU summary

## Power Management

- Power on reset(POR) initializes the register map with all the default values.
- POR can be executed by setting the `RST_SYS`-bit in the `SYS_TRIGGER` register.

### Power modes
- Three power modes: Normal, Low Power and Suspend mode.
- Selected by writing the `PWR_MODE` register
- Default -> Normal mode.  
(See Section 3.2 for details)

#### Normal Mode
- All sensors are switched on by default. 

#### Low Power Mode
- If no activity is detected for a configurable amount of time (default: 5s), BNO enters low power mode.
Only the accelerometer is active. 

#### Suspend Mode
- all sensors are paused and put into sleep mode. 

## Operation Modes
- default after power on is `CONFIGMODE`
- User can overwrite sensor settings when in `CONFIGMODE`
- In any mode the sensor data are available in the data register depending on the unit selected
- Operating mode can be selected using the `OPR_MODE` register

### Config Mode
- Used to configure the IMU 
- All data output is set to zero and sensor fusion is halted
- Only mode in which writable map entries can be changed

### Non-Fusion Modes
#### ACCONLY 
- Only accelerometer powered
- Raw accelerometer data
#### MAGONLY
- Stand-alone magnetometer
- all other sensors suspended
#### GYRONLY
- Stand-alone gyroscope
- all other sensors powered off
#### ACCMAG
- accelerometer and magnetometer are switched on
#### ACCGYRO
- accelerometer and gyroscope are switched on
#### MAGGYRO
- gyroscope and magnetometer are switched on
#### AMG (ACC-MAG-GYRO)
- all three sensors are powered on

### Fusion modes
- Calculate measures describing the orientation of the device in space
- distinguishing between non-absolute, relative and absolute orientation
- quaternion or euler angles
- accelerometer provides linear acceleration and gravity vector separately
#### IMU (Interial measurement unit)
- relative orientation from gyro and accelerometer
- Fast calculation
#### COMPASS
- calculation of geographic direction
#### M4G (Magnet for Gyroscope)
- similar to IMU mode bit instead of gyro it uses the magnetometer to detect rotation
- lower power consumption 
- no drift in data
- accuracy depends on surrounding magnetic field
#### NDOF_FMC_OFF
- same as NDOF but **Fast Magnetometer Calibration** is turned off.
#### NDOF 
- 9 DoF
- absolute orientation
- all three sensors
- fast calculation -> high data output rate

## Sensor Configuration
- sensors at default settings after power-on
- in any fusion mode

### Accelerometer configuration
- configuration possibilities are restricted when running in any of the fusion modes.
- can be changed by writing to the `ACC_Config` register
### Gyroscope configuration
- configuration possibilities restricted when running in any of the fusion modes.
- can be changed by writing to the `GYR_Config` register
### Magnetometer configuration
- configuration possibilities restricted when running in any of the fusion modes.
- can be changed by writing to the `MAG_Config` register.

## Output Data
### Unit selection
- measurement units depend on the bits set in the `UNIT_SEL` register

### Data output format
- Windows or Android => configured in the `UNIT_SEL` register

### Sensor calibration data
- Offset and radius data can be read from these registers (`x_OFFSET_x_xSB`)
#### Accelerometer offset
- can be configured using the `ACC_OFFSET_x_xSB` registers for each axis
- 2 bytes for each axis
- Configuration will take place only when the user writes the last byte (`ACC_OFFSET_Z_MSB`)
- maximum range offset depends on the configured G-range of the sensor
#### Magnetometer offset
- can be configured using the `MAG_OFFSET_x_xSB` registers for each axis
- 2 bytes for each axis
- Configuration will take place only when the user writes the last byte (`MAG_OFFSET_Z_MSB`)
#### Gyroscope offset
- can be configured using the `GYR_OFFSET_x_xSB` registers for each axis
- 2 bytes for each axis
- Configuration will take place only when the user writes the last byte (`GYR_OFFSET_Z_MSB`)
- maximum range depends on the configured dps-range of the sensor
#### Radius
- Can be configured using the `XXX_RADIUS_xSB` register where `XXX` can be `ACC` and `MAG`

### Output data registers
#### Acceleration data
- in non-fusion mode uncompensated acceleration data can be read for each axis X/Y/Z, from the appropriate registers
- in fusion mode the fusion algorithm output offset compensated acceleration data for each axis X/Y/Z, the output data can be read from the 
appropriate registers.
- `ACC_DATA_<axis>_xSB`
- 2 bytes per axis

#### Magnetic field strength
- in non-fusion mode uncompensated field strength data can be read for each axis X/Y/Z, from the appropriate registers
- in fusion mode the fusion algorithm output offset compensated magnetic field strength data for each axis X/Y/Z, the output data can be read from the 
appropriate registers.
- `MAG_DATA_<axis>_xSB`
- 2 bytes per axis

#### Angular velocity 
- in non-fusion mode uncompensated angular velocity (yaw rate) data can be read for each axis X/Y/Z, from the appropriate registers
- in fusion mode the fusion algorithm output offset compensated angular velocity data for each axis X/Y/Z, the output data can be read from the 
appropriate registers.
- `GYR_DATA_<axis>_xSB`
- 2 bytes per axis

#### Euler angles
- Orientation output only available in fusion mode
- can be read from the appropriate registers
- `EUL_<dof>_xSB`
- 2 bytes per dof

#### Orientation (Quaternion)
- Orientation output only available in fusion mode
- can be read from the appropriate registers
- `EUL_DATA_<dof>_xSB`
- 2 bytes per dof

#### Linear acceleration
- Orientation output only available in fusion mode
- can be read from the appropriate registers
- `LIA_DATA_<axis>_xSB`
- 2 bytes per axis

#### Gravity vector 
- Orientation output only available in fusion mode
- can be read from the appropriate registers
- `GRV_DATA_<axis>_xSB`
- 2 bytes per axis

#### Temperature
- can be read from the `TEMP` register
- possible to select the temperature sensor source with `TEMP_SOURCE` register


## Self-Test 
### Power on self test (POST)
- during power on a POST is executed. 
- checks if connected sensors and microcontroller are responding/functioning correctly
- results stored in the `ST_RESULT` register

### Build in Self Test (BIST)
- 

