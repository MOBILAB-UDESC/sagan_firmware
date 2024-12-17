#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "dwm_pico_AS5600.h"
#include "QuadratureEncoder.hpp"

#define SPACES "                              "

// Pins
#define ENCA_PIN 10 //B channel needs to be connected to the following pin (in this case pin 11)

int main()
{
    as5600_t as5600 = { 0 };
    static as5600_conf_t as5600_conf;
    static as5600_conf_t as5600_conf_bckp;

    float ppr = 1024.0; // pulses per revolution of the encoder
    float sampling_time = 1e-3;
    QuadratureEncoder encoder(ENCA_PIN, ppr);

    stdio_init_all();

    // Setup i2c
    i2c_init(i2c_default, 400 * 1000);

    // Setup as5600
    as5600_init(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, &as5600);

    printf("Hello, world!\n");

    while (true) {
        
        // encoder.update(sampling_time);

        printf("Raw value: %d | Non-raw value: %d%s\n", as5600_read_raw_angl(&as5600), as5600_read_angl(&as5600), SPACES);
        // printf("Position: %d | Velocity: %d%s\n", encoder.get_position(), encoder.get_velocity(), SPACES);
        sleep_ms(1);
    }
}