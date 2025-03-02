#include "motion_kalman.h"

    
void MotionKalman::init(float k0, float k1, float dt)
{
    this->k0    = k0;
    this->k1    = k1;
    this->dt    = dt;

    this->x1    = 0.0;
    this->x0    = 0.0;

    this->position_hat = 0.0;
    this->velocity_hat = 0.0;
}

void MotionKalman::step(float x) 
{
    this->x1 = this->x0;    
    this->x0 = x; 

    float v = (this->x0 - this->x1)/this->dt;

    this->position_hat = this->position_hat + this->k0*(x - this->position_hat);
    this->velocity_hat = this->velocity_hat + this->k1*(v - this->velocity_hat);
}

