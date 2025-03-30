#include "path_planner.h"
#include <drivers.h>


void PathPlanner::init()
{
    position_control.init();
       
    // forward acceleration G-force, mm/s^2
    this->a_max = 1.0*9.81*1000.0;
    this->a_min = -10.0*a_max;

    this->uv    = 0.0;
    
    this->time_now  = timer.get_time();
    this->time_prev = this->time_now;
}   
   

void PathPlanner::set_circle_motion(float radius, float speed)
{
    // obtain current state
    float distance  = this->position_control.get_distance();
    float angle     = this->position_control.get_angle();
  
    float dt = _get_dt();
    float dx = _smooth_speed(speed, dt);

    //calculate motion change
    float vc = dx;  
    float va = dx/radius;   

    float req_distance = distance + vc;
    float req_angle    = angle    + va;
   
    this->position_control.set_desired(req_distance, req_angle);
}



void PathPlanner::set_position(float req_distance, float req_angle)
{
    // obtain current state 
    float dt        = _get_dt();    

    float req_distance_ = _smooth_position(req_distance, 10000000, 0.75*a_max, dt);
    float req_angle_    = _smooth_angle(req_angle, 10000000, 0.01*a_max, dt);

    this->position_control.set_desired(req_distance_, req_angle_);  

    this->uv = 0.0;
}


void PathPlanner::set_circle_motion_trajectory(float radius, float speed)
{
    // obtain current state
    float distance  = this->position_control.get_distance();
    float angle     = this->position_control.get_angle();
    float velocity  = this->position_control.get_velocity();

    float dt = _get_dt();


    // estimate required velocity change (acceleration)
    // note : decelerration can be much faster 
    float acc_req = speed - velocity;
    acc_req = clip(acc_req, a_min*dt, a_max*dt);

    //antiwindup    
    //get_forward_saturation : returns 0 if none, +1 if upper limit, -1 if lower limit
    int saturation =this->position_control.get_forward_saturation();
    

    float uv_tmp = this->uv;

    // compute entire trajectory for MPC
    for (unsigned int n = 0; n < MPC_PREDCTION_HORIZON; n++)
    {
        if ((acc_req > 0 && saturation <= 0) || (acc_req < 0 && saturation >= 0))
        {
            uv_tmp = uv_tmp + 2.0*acc_req;
        }  

        //calculate motion change
        float dx = uv_tmp*dt;
        float vc = dx;  
        float va = dx/radius;   

        float req_distance = distance + vc;
        float req_angle    = angle    + va;
        
        // set trajectory for MPC
        this->position_control.set_desired_trajectory(n, req_distance, req_angle);

        if (n == 0)
        {
            this->uv = uv_tmp;
        }
    }
}


void PathPlanner::stop_position()
{
    // obtain current state
    float x = position_control.get_distance();
    float a = position_control.get_angle();

    set_position(x, a);
}



void PathPlanner::direct_control(float x_d, float a_d)
{
    // send to controller
    this->position_control.set_desired(x_d, a_d);
}

void PathPlanner::enable_lf()   
{
    this->position_control.lf_mode = true;
}

void PathPlanner::disable_lf()
{
    this->position_control.lf_mode = false;
}


float PathPlanner::_get_dt()
{
    this->time_prev = this->time_now;
    this->time_now  = timer.get_time(); 

    float dt = max(0.001f*(this->time_now - this->time_prev), 0.001f);

    return dt;
}


float PathPlanner::_smooth_speed(float desired_velocity, float dt)
{
    // obtain current state
    float x = position_control.get_distance();
    float v = position_control.get_velocity();

    // estimate required velocity change (acceleration)
    // note : decelerration can be much faster 
    float acc_req = desired_velocity - v;
    acc_req = clip(acc_req, a_min*dt, a_max*dt);

    //antiwindup    
    //get_forward_saturation : returns 0 if none, +1 if upper limit, -1 if lower limit
    int saturation = position_control.get_forward_saturation();

    if ((acc_req > 0 && saturation <= 0) || (acc_req < 0 && saturation >= 0))
    {
        uv = uv + 2.0*acc_req;
    }   

    return uv*dt;
}


float PathPlanner::_smooth_position(float x_req, float v_max, float acc_max, float dt)
{   
    // obtain current state
    float x = position_control.get_distance();
    float v = position_control.get_velocity();

    float dx    = x_req - x;
    float direction = sgn(dx);
    
    float v_req = dx/dt;      

    float dv    = clip(v_req - v, -acc_max, acc_max);
    float v_new = clip(v + dv, -v_max, v_max);

    float x_new = x + v_new * dt;

    return x_new;
}



float PathPlanner::_smooth_angle(float x_req, float v_max, float acc_max, float dt)
{   
    // obtain current state
    float x = position_control.get_angle();
    float v = position_control.get_angular_velocity();

    float dx    = x_req - x;    
    float direction = sgn(dx);
    
    float v_req = dx/dt;      

    float dv    = clip(v_req - v, -acc_max, acc_max);
    float v_new = clip(v + dv, -v_max, v_max);

    float x_new = x + v_new * dt;

    return x_new;
}