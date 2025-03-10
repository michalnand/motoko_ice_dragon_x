#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include <stdint.h>
#include "lqr.h"


// 4000us period = 250Hz loop
#define POSITION_CONTROL_DT     ((uint32_t)4000)

#define WHEEL_BRACE             ((float)78.0)
#define WHEEL_DIAMETER          ((float)28.0)


class PositionControl
{
    public:
        void init();
        void set_single_point(float distance, float angle);

        float get_distance();
        float get_angle();

        float get_velocity();
        float get_angular_velocity();

        
    public:
        void callback();

    private:
        void timer_init();
    
    private:
        LQR<4, 2> lqr;

    public:
        float distance_prev;
        float distance;
        float angle_prev;
        float angle;

    public:
        uint32_t steps;
};


#endif
