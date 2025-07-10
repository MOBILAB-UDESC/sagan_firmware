#include <stdio.h>
#include <utility>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "magnetic_encoder.h"
#include "quadrature_encoder.hpp"
#include "motor_driver.hpp"
#include "motor_speed_control.hpp"
#include "sagan_foc_control.hpp"

#define SPACES "                              "

// Pins
#define ENCA_PIN 10 // B channel needs to be connected to the following pin (in this case pin 11)
// Wheel Motor Driver Pins
#define ENABLE_WHEEL_DRIVER_PIN 18
#define CS_WHEEL_DRIVER_PIN 28
#define INPUT_A_WHEEL_DRIVER_PIN 21
#define INPUT_B_WHEEL_DRIVER_PIN 20
#define PWM_WHEEL_DRIVER_PIN 19


void motor_update(float PWM_INPUT, MotorDriver Motor_select)
{
    float PWM_VALUE = 0.0;
    if (PWM_INPUT >= 100 || PWM_INPUT <= -100)
    {
        if (PWM_INPUT > 0)
        {
            PWM_INPUT = 100;
            PWM_VALUE = round(PWM_INPUT * 255 / 100);
            Motor_select.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
            Motor_select.setMotorOutput(PWM_VALUE);
        }
        else
        {
            PWM_INPUT = -100;
            PWM_VALUE = round(-PWM_INPUT * 255 / 100);
            Motor_select.turnOnMotor(MotorDriver::CLOCKWISE);
            Motor_select.setMotorOutput(PWM_VALUE);
        }
    }
    else if (PWM_INPUT > 0)
    {
        PWM_VALUE = round(PWM_INPUT * 255 / 100);
        Motor_select.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        Motor_select.setMotorOutput(PWM_VALUE);
    }
    else
    {
        PWM_VALUE = round(-PWM_INPUT * 255 / 100);
        Motor_select.turnOnMotor(MotorDriver::CLOCKWISE);
        Motor_select.setMotorOutput(PWM_VALUE);
    }
}

#define motor_data_acquisition()                                                                                 \
    for (int pwm_valor = 0; pwm_valor <= 100; pwm_valor += 20)                                                   \
    {                                                                                                            \
        for (int i = 0; i < 1000; i++)                                                                           \
        {                                                                                                        \
            encoder.update(sampling_time);                                                                       \
            PWM_PERCENTAGE_WHEEL = pwm_valor;                                                                    \
            printf("Actual Vel: %.4f | Wheel PWM %.2f\n", encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, SPACES); \
            motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);                                                    \
            sleep_ms(5);                                                                                         \
        }                                                                                                        \
    }

#define motor_current_data_acquisition()                                                                                                                     \
    for (int pwm_valor = 0; pwm_valor <= 100; pwm_valor += 20)                                                                                               \
    {                                                                                                                                                        \
        for (int i = 0; i < 5000; i++)                                                                                                                       \
        {                                                                                                                                                    \
            encoder.update(sampling_time);                                                                                                                   \
            PWM_PERCENTAGE_WHEEL = pwm_valor;                                                                                                                \
            wheel_current = -0.1525 + wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;                                                               \
            printf("Actual Vel: %.4f | Wheel PWM %.2f | Wheel Motor Current : %.2f\n", encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, wheel_current, SPACES); \
            motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);                                                                                                \
            sleep_ms(1);                                                                                                                                     \
        }                                                                                                                                                    \
    }

int main()
{
    float sampling_time = 10e-3;
    QuadratureEncoder encoder(ENCA_PIN, 16, 30.0); // 30:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder (Helical Pinion)

    as5600_t as5600 = {0};
    static as5600_conf_t as5600_conf;
    static as5600_conf_t as5600_conf_bckp;

    stdio_init_all();

    // Setup i2c
    i2c_init(i2c_default, 400 * 1000);

    // Setup as5600
    as5600_init(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, &as5600);

    // Setup motor drivers
    MotorDriver wheel_driver(ENABLE_WHEEL_DRIVER_PIN, CS_WHEEL_DRIVER_PIN, INPUT_A_WHEEL_DRIVER_PIN, INPUT_B_WHEEL_DRIVER_PIN, PWM_WHEEL_DRIVER_PIN, MotorDriver::FULLBRIDGE);

    float PWM_PERCENTAGE_WHEEL = 0;

    float kp = 5;
    float ki = 8;
    float kd = 0;
    float N = 0;
    float targetVel = 0;

    int count = 0;
    uint64_t actual_time = 0;
    uint64_t prev_time = 0;

    float wheel_current = 0;
    float wheel_current_median = 0;
    float wheel_velocity = 0;

    SpeedControl MotorA(kp, ki, kd, N, sampling_time, 100);

    // bldc init
    stdio_init_all();

    SaganFOCControl::Config motor_config = {
        .pwm_pins = {7, 8, 9}, // PWM pins for phases U, V, W
        .enable_pin = 22,
        .pole_pairs = 7,
        .angle_p_gain = 500.0f, 
        .field_p_gain = 5.0f
    };

    SaganFOCControl BLDC(motor_config, as5600);
        
    BLDC.set_target_angle(M_PI / 2.0); // Set initial target

    while (true)
    {

        if (count == 0)
        {
            wheel_driver.turnOnMotor(MotorDriver::BRAKETOGND);
            wheel_driver.setMotorOutput(0);
            motor_update(0, wheel_driver);
            
            sleep_ms(1000);
            printf("Initializing code...\n");
            sleep_ms(1000);
            printf("3\n");
            sleep_ms(1000);
            printf("2\n");
            sleep_ms(1000);
            printf("1\n");
            sleep_ms(1000);

            BLDC.calibrate();

            printf("Finalized \n");
            sleep_ms(1000);

            count = 1;
        }
        float angle_target = M_PI / 2.0;
        BLDC.set_target_angle(angle_target); // Set initial target
        for (int index = 0; index <= 5000; index++){
            BLDC.update();
            printf("BLDC Position %.2f | BLDC Target %.2f\n", BLDC.get_position_rad(), angle_target, SPACES);
            sleep_ms(1);
        }
        
        // angle_target = 2 * M_PI; 
        // motor.set_target_angle(angle_target); 
        // for (int index = 0; index <= 5000; index++){
        //     motor.update();
        //     printf("BLDC Position %.2f | BLDC Target %.2f\n", motor.get_position_rad(), angle_target, SPACES);
        //     sleep_ms(1);
        // }

        // angle_target = 10 * M_PI; 
        // motor.set_target_angle(angle_target);
        // for (int index = 0; index <= 5000; index++){
        //     motor.update();
        //     printf("BLDC Position %.2f | BLDC Target %.2f\n", motor.get_position_rad(), angle_target, SPACES);
        //     sleep_ms(1);
        // }
        

        // wheel_driver.turnOnMotor(MotorDriver::BRAKETOVCC);
        // wheel_driver.setMotorOutput(0);
        // sleep_ms(10000);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//            MOTOR POLE TEST
//  This test is runned to know the motor poles number, if the correct value is asigned to the NUM_POLES variable
//  the motor will do one complete turn
// for (int i = 1; i <= POLES_NUM; i++)
// {
//     printf("Initialazing %i turn \n", i);
//     for (float intra_angle = 0.0; intra_angle <= 2*M_PI; intra_angle += M_PI/8)
//     {
//         alpha = intra_angle;
//         beta = intra_angle;
//         motor.space_vector_modulation(0.5 * cosf(alpha), 0.5 * sinf(beta));
//         sleep_ms(100);
//         angle = static_cast<float>(as5600_read_raw_angl(&as5600)) * 2 * M_PI / 4096.0;
//         printf("Position: %.2f rad (%.1f°), Electrical Angle: %.2f rad (%.1f°) \n", angle, angle * 180.0f / M_PI, intra_angle, intra_angle * 180.0f / M_PI);
//     }
// }
// printf("Finalized \n");
// sleep_ms(10000);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 // std::vector<float> velocity_vector = {0, 20, 30, 20, 10, 30, 10, 15};
        // printf("TargetVel;ActualVel;ControllerEffort,\n", SPACES);
        // for (int index = 0; index < velocity_vector.size(); index++)
        // {
        //     for (int tempo = 0; tempo <= 200; tempo++)
        //     {
        //         actual_time = time_us_64();
        //         encoder.update(static_cast<float>(actual_time - prev_time)/1000000.0);
        //         prev_time = actual_time;
                
        //         PWM_PERCENTAGE_WHEEL = MotorA.controlCalcRagazzini(velocity_vector[index], encoder.get_velocity());
        //         motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);
        //         printf("%.2f;%.2f;%.2f\n",velocity_vector[index], encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, SPACES);

        //         sleep_ms(10);
        //     }
        // }
        
        
        // std::vector<int> PWM_vector = {0, 20, 60, 80, 40, 80, 60, 10, 90, 10};
        // printf("PWM;ActualVel;\n", SPACES);
        // for (int index = 0; index < PWM_vector.size(); index++)
        // {
        //     PWM_PERCENTAGE_WHEEL = PWM_vector[index];
        //     for (int tempo = 0; tempo <= 200; tempo++)
        //     {
        //         actual_time = time_us_64();
        //         encoder.update(static_cast<float>(actual_time - prev_time)/1000000.0);
        //         prev_time = actual_time;
                
        //         motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);

        //         printf("%.2f;%.2f;\n",PWM_PERCENTAGE_WHEEL, encoder.get_velocity(), SPACES);

        //         sleep_ms(10);
        //     }
        // }

        // printf("TargeVel;ActualVel;EstimatedVel;ControllerEffort;Current;CurrentMedian;\n", SPACES);
        // for (float vel = 0; vel <= 35; vel += 5)
        // {
        //     targetVel = vel;
        //     for (int tempo = 0; tempo <= 500; tempo++)
        //     {
        //         for (int count = 0; count < 10; count++)
        //         {
        //             actual_time = time_us_64();
        //             encoder.update(static_cast<float>(actual_time - prev_time)/1000000.0);
        //             prev_time = actual_time;

        //             wheel_current = wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;
        //             wheel_current_median = MotorA.movingMedian(wheel_current - 0.40);
        //             wheel_velocity = MotorA.conversorCurrent2Velocity(wheel_current_median);
        //             //printf("Target Vel: %.2f | Actual Vel: %.2f | Estimated vel: %.2f | Wheel Controller Effort %.2f | Wheel motor current: %.2f | Wheel motor current median: %.2f \n", targetVel, encoder.get_velocity(), wheel_velocity, PWM_PERCENTAGE_WHEEL, wheel_current, wheel_current_median, SPACES);
        //             printf("%.2f;%.2f;%.2f;%.2f;%.2f;%.2f; \n", targetVel, encoder.get_velocity(), wheel_velocity, PWM_PERCENTAGE_WHEEL, wheel_current, wheel_current_median, SPACES);
        //             sleep_ms(1);
        //         }
        //         PWM_PERCENTAGE_WHEEL = MotorA.controlCalcPI(targetVel, wheel_velocity);
        //         //printf("%.2f,%.2f,%.2f\n",PWM_PERCENTAGE_WHEEL, wheel_current, encoder.get_velocity(), SPACES);
        //         motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);
        //     }
        // }
