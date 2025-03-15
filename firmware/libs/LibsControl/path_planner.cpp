#include "path_planner.h"
#include <drivers.h>


void PathPlanner::init()
{
    position_control.init();
   
    
    this->v_max     = 2700.0;
    this->w_max     = 20.0*360.0*PI/180.0;

    // forward acceleration G-force, mm/s^2
    this->a_max     = 1.0*9.81*1000.0;


    this->o_max     = 10000.0;   
    
    this->ux = 0.0;
    
    this->time_now  = timer.get_time();
    this->time_prev = this->time_now;
}
   


/*
    move robot forward, with desired velocity and turns by desired angle
*/
void PathPlanner::line_following(float desired_velocity, float desired_angle)
{
    float dt = _get_dt();

    // obtain current state
    float x = position_control.get_distance();
    float v = position_control.get_velocity();

      
    float acc_req = desired_velocity - v;
    acc_req = clip(acc_req, -4*a_max*dt, a_max*dt);


    //antiwindup
    //get_forward_saturation : returns 0 if none, +1 if upper limit, -1 if lower limit
    int saturation = position_control.get_forward_saturation();

    if ((acc_req > 0 && saturation <= 0) || (acc_req < 0 && saturation >= 0))
    {
        ux = ux + 4.0*acc_req;
    }   
    
    float x_ref = x + ux*dt;   

    // send to controller
    position_control.set_desired(x_ref, desired_angle);
} 


/*
void PathPlanner::line_following(float desired_velocity, float desired_angle)
{
    // obtain current state
    float x = position_control.get_distance();
    float v = position_control.get_velocity();

    // compute the required change in velocity.
    float delta_v_req = desired_velocity - v;
    
    // Limit the change in velocity to the maximum acceleration allowed.
    float max_delta_v = this->a_max*dt;
    float delta_v = clip(delta_v_req, -max_delta_v, max_delta_v);
    
    // Update the velocity.
    float v_new = v + delta_v;
    
    // Compute the new reference position using trapezoidal integration.
    //float x_ref = x + 0.5 * (v + v_new) * dt;
    float x_ref = x + v_new * dt;

    // send to controller
    position_control.set_desired(x_ref, desired_angle);
}   
*/

/*
    move robot to desired distance and set angle
*/
/*
void PathPlanner::point_following(float x_d, float a_d)
{
    float dt = _get_dt();

    float x = position_control.get_distance();
    float v = position_control.get_velocity()/dt;

    float a = position_control.get_angle();
    float w = position_control.get_angular_velocity()/dt;

    // Compute required velocity and angular rate to reach the desired state
    float v_req = (x_d - x) / dt; 
    float w_req = (a_d - a) / dt;

    // Limit velocity and angular rate
    v_req = clip(v_req, -v_max, v_max);
    w_req = clip(w_req, -w_max, w_max);

    // Compute acceleration and angular acceleration limits
    //float delta_v = clip(v_req - v, -a_max * dt, a_max * dt);
    //float delta_w = clip(w_req - w, -o_max * dt, o_max * dt);

    float delta_v = v_req - v;
    float delta_w = w_req - w;  

    // Update velocity and angular rate
    float v_new = v + delta_v;
    float w_new = w + delta_w;

    // Compute new reference position and angle using trapezoidal integration
    //float x_ref = x + 0.5 * (v + v_new) * dt;
    //float a_ref = a + 0.5 * (w + w_new) * dt;

    float x_ref = x + v_new*dt; 
    float a_ref = a + w_new*dt;

    // send to controller
    position_control.set_desired(x_ref, a_ref);
}
*/


void PathPlanner::point_following(float x_d, float a_d)
{
    float dt = _get_dt();

    float x = position_control.get_distance();
    float v = position_control.get_velocity();

    float a = position_control.get_angle(); 
    float w = position_control.get_angular_velocity();

    // required velocity
    float v_req = (x_d - x) / dt; 

    // limit velocity change by maximum acceleration
    float delta_v = clip(v_req - v, -a_max*dt, a_max*dt);

    // limit velocity value by maximum velocity
    float v_new = clip(v + delta_v, -v_max, v_max);  

    // compute requred distance
    float x_ref = x + v_new*dt; 

    // send to controller
    position_control.set_desired(x_ref, a_d);
}


void PathPlanner::direct_control(float x_d, float a_d)
{
    // send to controller
    position_control.set_desired(x_d, a_d);
}


float PathPlanner::_get_dt()
{
    this->time_prev = this->time_now;
    this->time_now  = timer.get_time(); 

    float dt = max(0.001f*(this->time_now - this->time_prev), 0.001f);

    return dt;
}