#ifndef MOTORSPEEDCONTROL_CPP
#define MOTORSPEEDCONTROL_CPP

#include "motor_speed_control.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

SpeedControl::SpeedControl(float kp, float ki, float kd, float N, float sampling_time, int saturation){
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->N = N;
    this->sampling_time = sampling_time;
    this->saturation = saturation;
    this->actualPos_prev = 0;
}

float SpeedControl::controlCalcPI(float targetVel, float actualVel){

    float e_i = 0;
    float e = 0;

    this->u_i_prev = u_i;
    this->u_prev = u;

    if( u_prev >= saturation && targetVel > actualVel){
        e_i = 0;
    } else if(u_prev <= -saturation && targetVel < actualVel) {
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

float SpeedControl::controlCalcPD(float targetPos, float actualPos){
    
    float e = targetPos - actualPos;

    this->u_d_prev = u_d;
    this->u_prev = u;

    float Td = 1 / kd;
    this->u_d = (Td / (Td + N * sampling_time) * u_d_prev) - ((kp * N * Td) / (Td + N * sampling_time) * (actualPos - actualPos_prev));
    float u_p = kp * e;
    this->u = u_p + u_d;

    this->actualPos_prev = actualPos;

    return u;
}



#endif