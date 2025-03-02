#ifndef _POSITION_CONTROL_H_
#define _POSITION_CONTROL_H_

#include <stdint.h>
#include "pid.h"

// 2500us periond = 400Hz loop
#define POSITION_CONTROL_DT     ((uint32_t)2500)

#define WHEEL_BRACE             ((float)78.0)
#define WHEEL_DIAMETER          ((float)28.0)

class PositionControl
{
    public:
        void init();
        void set_single_point(float distance, float angle);

        float get_distance();
        float get_velocity();
        float get_angle();
        float get_angular_velocity();


    public:
        void callback();

    private:
        void timer_init();

    
    private:
        float velocity_max;
        float distance, angle;

        PID distance_pid, angle_pid;
};


#endif
