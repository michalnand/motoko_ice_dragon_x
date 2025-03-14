#include "path_planner.h"

void PathPlanner::init()
{
    position_contol.init();
    
    this->dt = POSITION_CONTROL_DT*0.000001f;
    
    this->v_max     = 1.0f;
    this->w_max     = 1.0f;

    this->a_max     = 1.0f;
    this->o_max     = 1.0f;            
}
        
/*
    move robot forward, with desired velocity and turns by desired angle
*/
void PathPlanner::line_following(float desired_velocity, float desired_angle)
{
    // obtain current state
    float x = position_contol.get_distance();
    float v = position_contol.get_velocity();

    // compute the required change in velocity.
    float delta_v_req = desired_velocity - v;
    
    // Limit the change in velocity to the maximum acceleration allowed.
    float max_delta_v = this->a_max*dt;
    float delta_v = clip(delta_v_req, -max_delta_v, max_delta_v);
    
    // Update the velocity.
    float v_new = v + delta_v;
    
    // Compute the new reference position using trapezoidal integration.
    float x_ref = x + 0.5 * (v + v_new) * dt;

    // send to controller
    position_contol.set_desired(x_ref, desired_angle);
}   

/*
    move robot to desired distance and set angle
*/
void PathPlanner::point_following(float x_d, float a_d)
{
    float x = position_contol.get_distance();
    float v = position_contol.get_velocity();

    float a = position_contol.get_angle();
    float w = position_contol.get_angular_velocity();

    // Compute required velocity and angular rate to reach the desired state
    float v_req = (x_d - x) / dt; 
    float w_req = (a_d - a) / dt;

    // Limit velocity and angular rate
    v_req = clip(v_req, -v_max, v_max);
    w_req = clip(w_req, -w_max, w_max);

    // Compute acceleration and angular acceleration limits
    float delta_v = clip(v_req - v, -a_max * dt, a_max * dt);
    float delta_w = clip(w_req - w, -o_max * dt, o_max * dt);

    // Update velocity and angular rate
    float v_new = v + delta_v;
    float w_new = w + delta_w;

    // Compute new reference position and angle using trapezoidal integration
    float x_ref = x + 0.5 * (v + v_new) * dt;
    float a_ref = a + 0.5 * (w + w_new) * dt;

    // send to controller
    position_contol.set_desired(x_ref, a_ref);
}
