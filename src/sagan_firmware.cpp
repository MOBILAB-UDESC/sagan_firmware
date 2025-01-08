#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "magnetic_encoder.h"
#include "quadrature_encoder.hpp"
#include "motor_driver.hpp"
#include <utility>

#define SPACES "                              "

// Pins
#define ENCA_PIN 10 //B channel needs to be connected to the following pin (in this case pin 11)
// Wheel Motor Driver Pins
#define ENABLE_WHEEL_DRIVER_PIN 18
#define CS_WHEEL_DRIVER_PIN 28
#define INPUT_A_WHEEL_DRIVER_PIN 21
#define INPUT_B_WHEEL_DRIVER_PIN 20
#define PWM_WHEEL_DRIVER_PIN 19
// Steering Motor Driver Pins
#define ENABLE_STEERING_DRIVER_PIN 12
#define CS_STEERING_DRIVER_PIN 27
#define INPUT_A_STEERING_DRIVER_PIN 14
#define INPUT_B_STEERING_DRIVER_PIN 15
#define PWM_STEERING_DRIVER_PIN 13



int main()
{
    as5600_t as5600 = { 0 };
    static as5600_conf_t as5600_conf;
    static as5600_conf_t as5600_conf_bckp;

    float sampling_time = 1e-2;
    QuadratureEncoder encoder(ENCA_PIN, 16, 30.0); //30:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder (Helical Pinion)

    stdio_init_all();

    // Setup i2c
    i2c_init(i2c_default, 400 * 1000);

    // Setup as5600
    as5600_init(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, &as5600);

    // Setup motor drivers
    MotorDriver wheel_driver(ENABLE_WHEEL_DRIVER_PIN, CS_WHEEL_DRIVER_PIN, INPUT_A_WHEEL_DRIVER_PIN, INPUT_B_WHEEL_DRIVER_PIN, PWM_WHEEL_DRIVER_PIN, MotorDriver::FULLBRIDGE);    

    MotorDriver steering_driver(ENABLE_STEERING_DRIVER_PIN, CS_STEERING_DRIVER_PIN, INPUT_A_STEERING_DRIVER_PIN, INPUT_B_STEERING_DRIVER_PIN, PWM_STEERING_DRIVER_PIN, MotorDriver::FULLBRIDGE);  

    printf("Hello, world!\n");


    while (true) {
        
        //printf("Raw value: %d | Non-raw value: %d%s\n", as5600_read_raw_angl(&as5600), as5600_read_angl(&as5600), SPACES);
        //printf("Position: %d | Velocity: %f\n", encoder.get_count(), encoder.get_velocity(), SPACES);
        
        int PWM_PERCENTAGE = 0;
        int PWM_VALUE = 0;
        wheel_driver.setMotorOutput(PWM_VALUE);

        sleep_ms(2000);

        for(int j = 0; j <= 100; j += 20){
            PWM_PERCENTAGE = j;
            PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
            wheel_driver.turnOnMotor(MotorDriver::CLOCKWISE);
            wheel_driver.setMotorOutput(PWM_VALUE);
            for(int i = 0; i < 500; i++){
                encoder.update(sampling_time);
                printf("PWM: %d; Velocity: %f\n", PWM_PERCENTAGE, encoder.get_velocity(), SPACES);
                sleep_ms(10);
            }
        }
    }
}