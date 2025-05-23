#include "motion_kalman.h"

    
void MotionKalman::init(float k0, float k1, float dt)
{
    this->k0    = k0;
    this->k1    = k1;
    this->dt    = dt;

  
    this->position_hat = 0.0;
    this->velocity_hat = 0.0;
}

void MotionKalman::step(float position_measurement) 
{
    float prediction_error = position_measurement - this->position_hat;

    float position_new = this->position_hat + this->velocity_hat*this->dt + this->k0*prediction_error;
    float velocity_new = this->velocity_hat + this->k1*prediction_error;

    this->position_hat = position_new;
    this->velocity_hat = velocity_new;
}   




void MotionFilterEMA::init(float k0, float k1)
{
    this->k0 = k0;
    this->k1 = k1;
        
    this->position_hat = 0.0;
    this->velocity_hat = 0.0;
}

void MotionFilterEMA::step(float position_measurement)  
{
    float position_hat_new = (1.0 - this->k0)*this->position_hat + this->k0*position_measurement;
    float velocity_hat_new = (1.0 - this->k1)*this->velocity_hat + this->k1*(position_hat_new - this->position_hat);

    this->position_hat = position_hat_new;
    this->velocity_hat = velocity_hat_new;
}

