# STM32F4 Harware Abstraction Layer

This is a simple HAL for STM32F4 devices written in pure C.
I created it to learn more about how the inner livings of hardware and it's peripherals work.

All of this is still Work in Progress, and I may rewrite everything from scratch (Already done like 5 times) to 
add more abstractions and make the development and build process easier.

There is also a Bosch BNO055 Driver inside.

### Examples

GPIO PC13 high and low

```c
gpio_enable(PC13, GPIO_OUTPUT);

while (1) {
    // Pull Pin PC13 high for 1s
    gpio_high(PC13);
    delay_ms(1000);

    // Pull Pin PC13 low for 1s
    gpio_low(PC13);
    delay_ms(1000);
}
```

UART simple send:
```c
// UART init object
usart port;
port = (usart){
    .usart = USART2,            // UART port to use
    .baud = 115200,             // Baud rate
    .mode = USART_RX_TX_MODE,   // Mode 
    .interrupt_driven = true,   // Interrupt or polling
};

usart_init(&port);   
u32 counter = 0;

while (1) {
    usart_printf(port, "System on for %d sec\n", counter);
    delay_ms(1000);
}
```

Also the Clock configuration works:
```c
// Set system clock to 96MHz
rcc_system_clock_config(rcc_hse_25_mhz_to_96_mhz);   
```

I2C works too:
```c
// I2C1 init object
i2c1 = (i2c){
    .i2c = I2C1,            // I2C port to use
    .frequency = 16,        // frequency 
    .mode = I2C_STD_MODE,   // Mode
    .duty = 0,              // Duty cycle
};
// Initialize and setup all pins for I2C
// Also setup the clock for I2C
i2c_init(&i2c1);

// Write the temperature source into BNO sensor
i2c_write(i2c1, BNO_ADDR, BNO_TEMP_SOURCE, 0x00);
error i2c_err;
while (1) {
    i2c_err = i2c_read(i2c1, BNO_ADDR, BNO_OPR_MODE, as_ptr(opr_mode));
}
```

For more examples just look into the `main.c` for now.


### Contact/Contribution

Hit me up if you want to talk about the project or contribute.

E-Mail: [mailto:moonxraccoon@proton.me](moonxraccoon@proton.me)  
Discord: moonraccoon#1337
