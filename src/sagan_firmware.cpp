#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "magnetic_encoder.h"
#include "quadrature_encoder.hpp"
#include "motor_driver.hpp"
#include "motor_speed_control.hpp"
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

    float PWM_PERCENTAGE = 0;
    int PWM_VALUE = 0;

    float ANGLE_DEGREES = 0;
    float PREV_ANGLE_DEGREES = 0;
    float ANGLE_VELOCITY = 0;
    int ANGLE_ZERO = 0;

    float kp = 6.1279;
    float ki = 91.2238;
    float targetVel = 0;
    int count = 0;

    float kp2 = 6;
    float kd2 = 1.5134;
    float N = 6.3933;
    float targetAngle = 0;

    SpeedControl MotorA(kp, ki, 0, 0, sampling_time, 100);
    SpeedControl MotorB(kp2, 0, kd2, N, sampling_time, 100);

    while (true) {
        
        // encoder.update(sampling_time);
        //printf("Raw value: %d | Non-raw value: %d%s\n", as5600_read_raw_angl(&as5600), as5600_read_angl(&as5600), SPACES);
        //printf("Position: %d | Velocity: %f\n", encoder.get_count(), encoder.get_velocity(), SPACES);
        
        if (count == 0){
            sleep_ms(3000);
            wheel_driver.turnOnMotor(MotorDriver::BRAKETOGND);
            wheel_driver.setMotorOutput(0);
            steering_driver.turnOnMotor(MotorDriver::CLOCKWISE);
            steering_driver.setMotorOutput(255);
            sleep_ms(1000);            
            steering_driver.turnOnMotor(MotorDriver::BRAKETOGND);
            steering_driver.setMotorOutput(0);
            
            ANGLE_ZERO = as5600_read_raw_angl(&as5600);
            count = 1;
        }

        for(int j = 0; j <= 180; j += 10){

            targetAngle = j;
            
            for(int i = 0; i < 500; i++){
                ANGLE_DEGREES = static_cast<float>(as5600_read_raw_angl(&as5600) - ANGLE_ZERO) * 360.0 / 4096.0;
                PWM_PERCENTAGE = MotorB.controlCalcPD(targetAngle, ANGLE_DEGREES);
                printf("Target Angle: %f |  Actual Angle: %f | Controller Effort: %f\n", targetAngle, ANGLE_DEGREES, PWM_PERCENTAGE, SPACES);

                if (PWM_PERCENTAGE >= 100 || PWM_PERCENTAGE <= -100){
                    if(PWM_PERCENTAGE > 0){
                        PWM_PERCENTAGE = 100;
                        PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
                        steering_driver.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
                        steering_driver.setMotorOutput(PWM_VALUE);
                    } else {
                        PWM_PERCENTAGE = -100;
                        PWM_VALUE = round(-PWM_PERCENTAGE * 255 / 100);
                        steering_driver.turnOnMotor(MotorDriver::CLOCKWISE);
                        steering_driver.setMotorOutput(PWM_VALUE);
                    } 
                } else if( PWM_PERCENTAGE > 0){
                    PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
                    steering_driver.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
                    steering_driver.setMotorOutput(PWM_VALUE);
                } else {
                    PWM_VALUE = round(-PWM_PERCENTAGE * 255 / 100);
                    steering_driver.turnOnMotor(MotorDriver::CLOCKWISE);
                    steering_driver.setMotorOutput(PWM_VALUE);
                }
                sleep_ms(10);
            }
        }

        steering_driver.turnOnMotor(MotorDriver::BRAKETOGND);
        steering_driver.setMotorOutput(0);
        sleep_ms(5000);
        


        // for(float j = 40.0; j <= 100.0; j += 20.0){
        //     steering_driver.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        //     PWM_PERCENTAGE = j;
        //     PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
        //     steering_driver.setMotorOutput(PWM_VALUE);
            
        //     while ( ANGLE_DEGREES < 180){
        //         PREV_ANGLE_DEGREES = ANGLE_DEGREES;
        //         ANGLE_DEGREES = static_cast<float>(as5600_read_raw_angl(&as5600) - ANGLE_ZERO) * 360.0 / 4096.0;
        //         ANGLE_VELOCITY = (ANGLE_DEGREES - PREV_ANGLE_DEGREES) / sampling_time;
        //         printf("Angle in Degrees: %f || Angle velocity: %f || PWM Value: %f\n", ANGLE_DEGREES,ANGLE_VELOCITY, PWM_PERCENTAGE, SPACES);
        //         sleep_ms(10);
        //     }

        //     wheel_driver.turnOnMotor(MotorDriver::BRAKETOGND);
        //     wheel_driver.setMotorOutput(0);
        //     sleep_ms(1000);

        //     steering_driver.turnOnMotor(MotorDriver::CLOCKWISE);
        //     PWM_PERCENTAGE = 40;
        //     PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
        //     steering_driver.setMotorOutput(PWM_VALUE);

        //     while ( ANGLE_DEGREES > 1){
        //         PREV_ANGLE_DEGREES = ANGLE_DEGREES;
        //         ANGLE_DEGREES = static_cast<float>(as5600_read_raw_angl(&as5600) - ANGLE_ZERO) * 360.0 / 4096.0;
        //         ANGLE_VELOCITY = (ANGLE_DEGREES - PREV_ANGLE_DEGREES) / sampling_time;
        //         //printf("Angle in Degrees: %f || Angle velocity: %f || PWM Value: %f\n", ANGLE_DEGREES,ANGLE_VELOCITY, PWM_PERCENTAGE, SPACES);
        //     }
            
        //     wheel_driver.turnOnMotor(MotorDriver::BRAKETOGND);
        //     wheel_driver.setMotorOutput(0);
        //     sleep_ms(1000);
        // }

        // encoder.update(sampling_time);
        // PWM_PERCENTAGE = MotorA.controlCalcPI(targetVel, encoder.get_velocity());

        // printf("Target Velocity: %f |  Actual Velocity: %f | Controller Effort: %f\n", targetVel, encoder.get_velocity(), PWM_PERCENTAGE, SPACES);
        
        // if (PWM_PERCENTAGE >= 100 || PWM_PERCENTAGE <= -100){
        //     if(PWM_PERCENTAGE > 0){
        //         PWM_PERCENTAGE = 100;
        //         PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
        //         wheel_driver.turnOnMotor(MotorDriver::CLOCKWISE);
        //         wheel_driver.setMotorOutput(PWM_VALUE);
        //     } else {
        //         PWM_PERCENTAGE = -100;
        //         PWM_VALUE = round(-PWM_PERCENTAGE * 255 / 100);
        //         wheel_driver.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        //         wheel_driver.setMotorOutput(PWM_VALUE);
        //     } 
        // } else if( PWM_PERCENTAGE > 0){
        //     PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
        //     wheel_driver.turnOnMotor(MotorDriver::CLOCKWISE);
        //     wheel_driver.setMotorOutput(PWM_VALUE);
        // } else {
        //     PWM_VALUE = round(-PWM_PERCENTAGE * 255 / 100);
        //     wheel_driver.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        //     wheel_driver.setMotorOutput(PWM_VALUE);
        // }

        // sleep_ms(10);

        // for(int j = 0; j <= 40; j += 10){

        //     targetVel = j;
            
        //     for(int i = 0; i < 500; i++){
        //         encoder.update(sampling_time);
        //         PWM_PERCENTAGE = MotorA.controlCalcPI(targetVel, encoder.get_velocity());
        //         printf("Target Velocity: %f |  Actual Velocity: %f | Controller Effort: %f\n", targetVel, encoder.get_velocity(), PWM_PERCENTAGE, SPACES);

        //         if (PWM_PERCENTAGE >= 100 || PWM_PERCENTAGE <= -100){
        //             if(PWM_PERCENTAGE > 0){
        //                 PWM_PERCENTAGE = 100;
        //                 PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
        //                 wheel_driver.turnOnMotor(MotorDriver::CLOCKWISE);
        //                 wheel_driver.setMotorOutput(PWM_VALUE);
        //             } else {
        //                 PWM_PERCENTAGE = -100;
        //                 PWM_VALUE = round(-PWM_PERCENTAGE * 255 / 100);
        //                 wheel_driver.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        //                 wheel_driver.setMotorOutput(PWM_VALUE);
        //             } 
        //         } else if( PWM_PERCENTAGE > 0){
        //             PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
        //             wheel_driver.turnOnMotor(MotorDriver::CLOCKWISE);
        //             wheel_driver.setMotorOutput(PWM_VALUE);
        //         } else {
        //             PWM_VALUE = round(-PWM_PERCENTAGE * 255 / 100);
        //             wheel_driver.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        //             wheel_driver.setMotorOutput(PWM_VALUE);
        //         }
        //         sleep_ms(10);
        //     }
        // }

        // wheel_driver.setMotorOutput(0);
        // sleep_ms(2000);
    }
        
        // wheel_driver.setMotorOutput(PWM_VALUE);
        // sleep_ms(2000);

        // for(int j = 0; j <= 40; j += 10){
        //     PWM_PERCENTAGE = j;
        //     PWM_VALUE = round(PWM_PERCENTAGE * 255 / 100);
        //     wheel_driver.turnOnMotor(MotorDriver::CLOCKWISE);
        //     wheel_driver.setMotorOutput(PWM_VALUE);
        //     for(int i = 0; i < 500; i++){
        //         encoder.update(sampling_time);
        //         printf("PWM: %d; Velocity: %f\n", PWM_PERCENTAGE, encoder.get_velocity(), SPACES);
        //         sleep_ms(10);
        //     }
        // }
}
