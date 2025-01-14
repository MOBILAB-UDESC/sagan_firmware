#ifndef MOTORSPEEDCONTROL_HPP
#define MOTORSPEEDCONTROL_HPP

class SpeedControl
{
    public:

    SpeedControl(float kp, float ki, float kd, float N, float sampling_time, int saturation);

    float controlCalcPI(float targetVel, float actualVel);

    float controlCalcPD(float targetPos, float actualPos);

    private:
        float kp;
        float ki;
        float kd;
        float N;
        float sampling_time;
        float u_i;
        float u_i_prev;
        float u;
        float u_prev;
        float u_d;
        float u_d_prev;
        float actualPos_prev;
        int saturation;
};


#endif