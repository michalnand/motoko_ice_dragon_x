#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include "position_control.h"
#include "fmath.h"

#include "s_curve.h"


class PathPlanner
{
    public:
        void init();

        void set_position(float distance, float angle);

        void set_circle_motion(float r_req, float v_req);
        void set_circle_motion_trajectory(float radius, float speed);

        void enable_lf();
        void disable_lf();

    private:
        float _get_dt();

    public:
        PositionControl position_control;

    public:
        float dt, uv;
        float time_now, time_prev;

        float acc_max, tau_f, v_smooth;
        float acc_w_max, tau_t, w_smooth;
        
};


#endif
