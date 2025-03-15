#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include "position_control.h"
#include "fmath.h"

class PathPlanner
{
    public:
        void init();
        
        /*
            move robot forward, with desired velocity and turns by desired angle
        */
        void line_following(float desired_velocity, float desired_angle);
        
        /*
            move robot to desired distance and set angle
        */
        void point_following(float x_d, float a_d);
        

        void direct_control(float x_d, float a_d);

    private:
        float _get_dt();


    public:
        PositionControl position_control;

    private:
        uint32_t time_now, time_prev;

        float v_max, w_max;
        float a_max, o_max;
        
        float ux;
};

#endif