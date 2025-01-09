#ifndef MOTORSPEEDCONTROL_HPP
#define MOTORSPEEDCONTROL_HPP

class SpeedControl
{
    public:

    SpeedControl(float kp, float ki, float sampling_time, int saturation);

    float controlActionCalc(float targetVel, float actualVel);

    private:
        float kp;
        float ki;
        float sampling_time;
        float u_i;
        float u_i_prev;
        float u;
        float u_prev;
        int saturation;
};


#endif