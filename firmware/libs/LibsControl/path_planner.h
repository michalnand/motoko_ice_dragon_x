#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include "position_control.h"
#include "fmath.h"

class PathPlanner
{
    public:
        void init();
        
        void set_circle_motion(float radius, float speed);
        void set_circle_motion_trajectory(float radius, float speed);

        void set_position(float req_distance, float req_angle);
        void stop_position();

        void direct_control(float x_d, float a_d);
 
        void enable_lf();
        void disable_lf();


    private:
        float _get_dt();
        float _smooth_speed(float desired_velocity, float dt);

        float _smooth_position(float x_req, float v_max, float acc_max, float dt);
        float _smooth_angle(float x_req, float v_max, float acc_max, float dt);



    public:
        PositionControl position_control;

    private:
        uint32_t time_now, time_prev;

        float a_min, a_max;
        float uv;
};

#endif