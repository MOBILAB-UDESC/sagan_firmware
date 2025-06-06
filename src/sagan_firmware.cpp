#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "magnetic_encoder.h"
#include "quadrature_encoder.hpp"
#include "motor_driver.hpp"
#include "motor_speed_control.hpp"
#include <utility>
#include <stdint.h>

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

void motor_update(float PWM_INPUT, MotorDriver Motor_select){
    float PWM_VALUE = 0.0;
    if (PWM_INPUT >= 100 || PWM_INPUT <= -100){
        if(PWM_INPUT > 0){
            PWM_INPUT = 100;
            PWM_VALUE = round(PWM_INPUT * 255 / 100);
            Motor_select.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
            Motor_select.setMotorOutput(PWM_VALUE);
        } else {
            PWM_INPUT = -100;
            PWM_VALUE = round(-PWM_INPUT * 255 / 100);
            Motor_select.turnOnMotor(MotorDriver::CLOCKWISE);
            Motor_select.setMotorOutput(PWM_VALUE);
        } 
    } else if( PWM_INPUT > 0){
        PWM_VALUE = round(PWM_INPUT * 255 / 100);
        Motor_select.turnOnMotor(MotorDriver::COUNTERCLOCKWISE);
        Motor_select.setMotorOutput(PWM_VALUE);
    } else {
        PWM_VALUE = round(-PWM_INPUT * 255 / 100);
        Motor_select.turnOnMotor(MotorDriver::CLOCKWISE);
        Motor_select.setMotorOutput(PWM_VALUE);
    }
}

#define motor_data_acquisition() \
    for(int pwm_valor = 0; pwm_valor <= 100; pwm_valor += 20){  \
        for(int i = 0; i < 1000; i++){ \
            encoder.update(sampling_time); \
            PWM_PERCENTAGE_WHEEL = pwm_valor; \
            printf("Actual Vel: %.4f | Wheel PWM %.2f\n", encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, SPACES); \
            motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver); \
            sleep_ms(5); \
        } \
    } \

#define motor_current_data_acquisition() \
    for(int pwm_valor = 0; pwm_valor <= 100; pwm_valor += 20){  \
        for(int i = 0; i < 5000; i++){ \
            encoder.update(sampling_time); \
            PWM_PERCENTAGE_WHEEL = pwm_valor; \
            wheel_current = -0.1525 + wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0; \
            printf("Actual Vel: %.4f | Wheel PWM %.2f | Wheel Motor Current : %.2f\n", encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, wheel_current, SPACES); \
            motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver); \
            sleep_ms(1); \
        } \
    } \

    int main()
{
    as5600_t as5600 = { 0 };
    static as5600_conf_t as5600_conf;
    static as5600_conf_t as5600_conf_bckp;

    float sampling_time = 1e-3;
    QuadratureEncoder encoder(ENCA_PIN, 16, 30.0); //30:1 Metal Gearmotor 37Dx68L mm 12V with 64 CPR Encoder (Helical Pinion)

    stdio_init_all();

    // Setup i2c
    i2c_init(i2c_default, 400 * 1000);

    // Setup as5600
    as5600_init(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, &as5600);

    // Setup motor drivers 
    MotorDriver steering_driver(ENABLE_STEERING_DRIVER_PIN, CS_STEERING_DRIVER_PIN, INPUT_A_STEERING_DRIVER_PIN, INPUT_B_STEERING_DRIVER_PIN, PWM_STEERING_DRIVER_PIN, MotorDriver::FULLBRIDGE);  
    MotorDriver wheel_driver(ENABLE_WHEEL_DRIVER_PIN, CS_WHEEL_DRIVER_PIN, INPUT_A_WHEEL_DRIVER_PIN, INPUT_B_WHEEL_DRIVER_PIN, PWM_WHEEL_DRIVER_PIN, MotorDriver::FULLBRIDGE);

    float PWM_PERCENTAGE_STEERING = 0;
    float PWM_PERCENTAGE_WHEEL = 0;

    float ANGLE_DEGREES = 0;
    float PREV_ANGLE_DEGREES = 0;
    float ANGLE_VELOCITY = 0;
    int ANGLE_ZERO = 0;

    float kp = 6.1279;
    float ki = 91.2238;
    float kd = 0;
    float N = 0;
    float targetVel = 0;

    float kp2 = 15;
    float ki2 = 10;
    float kd2 = 0;
    float N2= 0;
    float targetAngle = 0;

    int count = 0;

    float wheel_current = 0;
    float steering_current = 0;

    SpeedControl MotorA(kp, ki, kd, N, sampling_time, 100);
    SpeedControl MotorB(kp2, ki2, kd2, N2, sampling_time, 100);

    while (true) {
        
        if (count == 0){
            sleep_ms(1000);
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

        motor_current_data_acquisition();



        // for(int j = 0; j <= 180; j += 60){

        //     targetAngle = j;
        //     targetVel = j / 5;
            
        //     for(int i = 0; i < 500; i++){
        //         ANGLE_DEGREES = static_cast<float>(as5600_read_raw_angl(&as5600) - ANGLE_ZERO) * 360.0 / 4096.0;
        //         encoder.update(sampling_time);
                
        //         PWM_PERCENTAGE_WHEEL = MotorA.controlCalcPI(targetVel, encoder.get_velocity());
        //         PWM_PERCENTAGE_STEERING = MotorB.controlCalcPI(targetAngle, ANGLE_DEGREES);

        //         wheel_current = -0.1525 + wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;
        //         steering_current = -0.1525 + steering_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;

        //         //printf("Target Angle: %.2f |  Actual Angle: %.2f |  Steering Controller Effort: %.2f | Target Vel: %.2f | Actual Vel: %.2f | Wheel Controller Effort %.2f\n", targetAngle, ANGLE_DEGREES, PWM_PERCENTAGE_STEERING, targetVel, encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, SPACES);
        //         //printf("Wheel Motor Current : %f | Steering Motor Current %f\n", wheel_current, steering_current, SPACES);
        //         printf("Wheel Motor Current : %f | Wheel PWM: %.2f | Steering Motor Current %f | Steering PWM: %.2f\n", wheel_current, PWM_PERCENTAGE_WHEEL, steering_current, PWM_PERCENTAGE_STEERING, SPACES);
        //         motor_update(PWM_PERCENTAGE_STEERING, steering_driver);
        //         motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);                  
        //         sleep_ms(10);
        //     }
        // }

        // for(int j = 180; j >= 0; j -= 60){

        //     targetAngle = j;
        //     targetVel = j / 5;
            
        //     for(int i = 0; i < 500; i++){
        //         ANGLE_DEGREES = static_cast<float>(as5600_read_raw_angl(&as5600) - ANGLE_ZERO) * 360.0 / 4096.0;
        //         encoder.update(sampling_time);
                
        //         PWM_PERCENTAGE_WHEEL = MotorA.controlCalcPI(targetVel, encoder.get_velocity());
        //         PWM_PERCENTAGE_STEERING = MotorB.controlCalcPI(targetAngle, ANGLE_DEGREES);

        //         wheel_current = wheel_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;
        //         steering_current = steering_driver.checkMotorCurrentDraw() * 11370.0 / 1500.0;
                
        //         //printf("Target Angle: %.2f |  Actual Angle: %.2f |  Steering Controller Effort: %.2f | Target Vel: %.2f | Actual Vel: %.2f | Wheel Controller Effort %.2f\n", targetAngle, ANGLE_DEGREES, PWM_PERCENTAGE_STEERING, targetVel, encoder.get_velocity(), PWM_PERCENTAGE_WHEEL, SPACES);
        //         printf("Wheel Motor Current : %f | Wheel PWM: %.2f | Steering Motor Current %f | Steering PWM: %.2f\n", wheel_current, PWM_PERCENTAGE_WHEEL, steering_current, PWM_PERCENTAGE_STEERING, SPACES);
        //         motor_update(PWM_PERCENTAGE_STEERING, steering_driver);
        //         motor_update(PWM_PERCENTAGE_WHEEL, wheel_driver);                  
        //         sleep_ms(10);
        //     }
        // }

        steering_driver.turnOnMotor(MotorDriver::BRAKETOGND);
        steering_driver.setMotorOutput(0);
        wheel_driver.turnOnMotor(MotorDriver::BRAKETOVCC);
        wheel_driver.setMotorOutput(0);
        sleep_ms(5000);
    }
}
