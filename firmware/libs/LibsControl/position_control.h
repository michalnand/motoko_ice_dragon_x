#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include <stdint.h>

#include "lqr.h"
#include "lqri.h"   

// 4000us period = 250Hz loop
#define POSITION_CONTROL_DT     ((uint32_t)4000)

#define WHEEL_BRACE             ((float)78.0)
#define WHEEL_DIAMETER          ((float)28.0)


class PositionControl
{   
    public:
        void init();
        void set_desired(float distance, float angle);

        float get_distance();
        float get_angle();

        float get_velocity();
        float get_angular_velocity();

        float get_u_forward();
        float get_u_turn();

        int get_forward_saturation();
        int get_turn_saturation();

        
    public:
        void callback();

    private:
        void timer_init();
    
    private:
        LQR<4, 2> controller;   
        //LQRI<4, 2> controller;   
        

    private:
        float distance;
        float angle;
        float velocity;
        float angular_rate;

        float u_forward,  u_turn;

    private:
        float line_angle, line_angle_prev;
    
    public:
        bool  lf_mode;

    public:
        uint32_t steps;
};


#endif
