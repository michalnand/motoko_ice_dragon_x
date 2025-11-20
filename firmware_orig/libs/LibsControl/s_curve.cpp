#include "s_curve.h"

void SCurve::init(float tau1, float tau2)
{
    this->_x1 = 0.0f;
    this->_x2 = 0.0f;
 
    this->tau1 = tau1;
    this->tau2 = tau2;

    this->reset();
}

void SCurve::reset(float x_init, float v_init)
{
    this->_x1           = x_init;
    this->_x2           = x_init;
}

float SCurve::smooth_position(float x_req, float x_meas, float v_meas, float dt)
{
    // estimate filter alpha from time constnants and dt step
    float alpha1 = dt / (this->tau1 + dt);
    float alpha2 = dt / (this->tau2 + dt);  

    // first state of position smoothing
    this->_x1 = (1.0 - alpha1)*this->_x1 + alpha1*x_req;

    // second state
    float x_error = this->_x1 - x_meas;
    this->_x2 = (1.0 - alpha2)*this->_x2 + alpha2*(x_error - 0.2 * v_meas);

    return this->_x2;
}

/*
float SCurve::smooth_velocity(float v_req, float v_meas)
{
    if (!this->_initialized)
    {
        this->reset(x_meas, v_meas);
    }

    // Stage 1: Smooth requested velocity
    this->_v1 += this->alpha1 * (v_req - this->_v1);

    // Stage 2: Smooth again with damping from current velocity
    v_error = this->_v1 - v_meas;
    damping = 0.2 * v_meas;

    this->_v2 += this->alpha2 * (v_error - damping);

    return this->_v2
}
*/
