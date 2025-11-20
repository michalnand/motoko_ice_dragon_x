#include "path_planner.h"
#include <drivers.h>


void PathPlanner::init()
{
    position_control.init();

    this->acc_max   = 3*9.81*1000.0;
    this->acc_min   = -30*9.81*1000.0;

    this->tau_f     = 0.2;  
    this->v_smooth  = 0.0;  

    this->acc_w_max = 2*9.81*1000.0;

    this->tau_t     = 0.05;
    this->w_smooth  = 0.0;

    this->uv = 0.0;

    this->time_now  = timer.get_time();
    this->time_prev = this->time_now;
}


void PathPlanner::set_position(float x_req, float a_req)
{
    // obtain time interval
    float dt    = this->_get_dt();

    // compute alpha from time constants and dt step
    float alpha_f = dt / (this->tau_f + dt);
    float alpha_t = dt / (this->tau_t + dt);

    // obtain state
    float x     = position_control.get_distance();
    float v     = position_control.get_velocity();
    float a     = position_control.get_angle();
    float w     = position_control.get_angular_velocity();

    // required position change
    float dx    = x_req - x;
    float v_req = dx/dt;    
    
    float v_new    = v + clip(v_req - v, acc_min, acc_max);
    this->v_smooth = (1.0 - alpha_f)*this->v_smooth + alpha_f*v_new;

    float v_tmp    = min(abs(v_req), abs(this->v_smooth))*sgn(v_req);

    float x_new = x + v_tmp*dt;



    // required angle change
    float da    = a_req - a;
    float w_req = da/dt;    
    
    float w_new    = w + clip(w_req - w, -acc_w_max, acc_w_max);
    this->w_smooth = (1.0 - alpha_t)*this->w_smooth + alpha_t*w_new;

    float w_tmp    = min(abs(w_req), abs(this->w_smooth))*sgn(w_req);

    float a_new = a + w_tmp*dt;

    position_control.set_desired(x_new, a_new);

    this->uv = 0.0;
}


void PathPlanner::set_circle_motion(float r_req, float v_req)
{   
    // obtain time interval
    float dt = _get_dt();

    // obtain current state
    float x         = this->position_control.get_distance();
    float v         = position_control.get_velocity();
    float angle     = this->position_control.get_angle();

    // estimate required velocity change (acceleration)
    // note : decelerration can be much faster 
    float acc_req = v_req - v;

    // accelearation limit
    acc_req = clip(acc_req, acc_min*dt, acc_max*dt);


    //antiwindup    
    //get_forward_saturation : returns 0 if none, +1 if upper limit, -1 if lower limit
    int saturation = position_control.get_forward_saturation();

    if ((acc_req > 0 && saturation <= 0) || (acc_req < 0 && saturation >= 0))
    {
        this->uv = this->uv + acc_req;
    }   


    //calculate motion change
    float vc = this->uv;  
    float va = this->uv/r_req;   

    float req_x     = x     + vc*dt;
    float req_angle = angle + va*dt;

    this->position_control.set_desired(req_x, req_angle);
}

void PathPlanner::set_circle_motion_trajectory(float r_req, float v_req)
{
    // obtain time interval
    float dt = _get_dt();

    // obtain current state
    float x         = this->position_control.get_distance();
    float v         = position_control.get_velocity();
    float angle     = this->position_control.get_angle();

    // estimate required velocity change (acceleration)
    // note : decelerration can be much faster 
    float acc_req = v_req - v;

    // accelearation limit
    acc_req = clip(acc_req, acc_min*dt, acc_max*dt);

    //antiwindup    
    //get_forward_saturation : returns 0 if none, +1 if upper limit, -1 if lower limit
    int saturation = position_control.get_forward_saturation();

    float uv = this->uv;

    for (unsigned int n = 0; n < MPC_PREDCTION_HORIZON; n++)
    {
        if ((acc_req > 0 && saturation <= 0) || (acc_req < 0 && saturation >= 0))
        {
            uv = uv + acc_req;
        }     

        //calculate motion change trajectory
        float vc = uv;  
        float va = uv/r_req;    

        float req_x     = x     + vc*dt;
        float req_angle = angle + va*dt;  

        this->position_control.set_desired_trajectory(n, req_x, req_angle);

        // first step is true value of v_smooth
        if (n == 0)
        {
            this->uv = uv;
        }   
    }
}

void PathPlanner::enable_lf()
{
    position_control.lf_mode = true;
}

void PathPlanner::disable_lf()
{
    position_control.lf_mode = true;
}


float PathPlanner::_get_dt()
{
    this->time_prev = this->time_now;
    this->time_now  = timer.get_time(); 

    float dt = max(0.001f*(this->time_now - this->time_prev), 0.00001f);

    this->dt = dt;
    return dt;
}
