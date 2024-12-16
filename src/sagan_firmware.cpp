#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "dwm_pico_AS5600.h"
#include "QuadratureEncoder.hpp"

#define SPACES "                              "

// Pins
#define ENCA_PIN 10

int main()
{
    as5600_t as5600 = { 0 };
    static as5600_conf_t as5600_conf;
    static as5600_conf_t as5600_conf_bckp;

    stdio_init_all();

    // Setup i2c
    i2c_init(i2c_default, 400 * 1000);

    // Setup as5600
    as5600_init(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, &as5600);

    printf("Hello, world!\n");

    while (true) {
        
        auto value = as5600_read_raw_angl(&as5600);

        printf("Raw value: %d | Non-raw value: %d%s\r", value, as5600_read_angl(&as5600), SPACES);
        sleep_ms(1000);
    }
}