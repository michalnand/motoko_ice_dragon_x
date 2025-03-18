#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include "position_control.h"
#include "fmath.h"

class PathPlanner
{
    public:
        void init();
        
        void set_circle_motion(float radius, float speed);
        void direct_control(float x_d, float a_d);

    private:
        float _get_dt();
        float _smooth_speed(float desired_velocity, float dt);



    public:
        PositionControl position_control;

    private:
        uint32_t time_now, time_prev;

        float a_min, a_max;
        float uv;
};

#endif