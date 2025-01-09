#ifndef MOTORSPEEDCONTROL_CPP
#define MOTORSPEEDCONTROL_CPP

#include "motor_speed_control.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

SpeedControl::SpeedControl(float kp, float ki, float sampling_time, int saturation){
    this->kp = kp;
    this->ki = ki;
    this->sampling_time = sampling_time;
    this->saturation = saturation;
    printf("Speed Control Inicialized");
}

float SpeedControl::controlActionCalc(float targetVel, float actualVel){

    float e_i = 0;
    float e = 0;

    this->u_i_prev = u_i;
    this->u_prev = u;

    if( u_prev >= saturation && targetVel > actualVel){
        e_i = 0;
    } else {
        e_i = targetVel - actualVel;
    }

    e = targetVel - actualVel;

    this->u_i = u_i_prev + (kp * sampling_time * ki) * e_i;
    float u_p = kp * e;
    this->u = u_p + u_i;

    return u;
}

#endif