#include <pid.h>


PID::PID()
{
    init(0, 0, 0, 0, 0);
}

PID::PID(float kp, float ki, float kd, float antiwindup, float du_max)
{
    init(kp, ki, kd, antiwindup, du_max);
}

void PID::init(float kp, float ki, float kd, float antiwindup, float du_max)
{   
    this->k0 = kp + ki + kd;
    this->k1 = -kp -2.0*kd;
    this->k2 = kd;

    this->u = 0;
    this->antiwindup = antiwindup;
    this->du_max     = du_max;

    this->e0 = 0.0;
    this->e1 = 0.0;     
    this->e2 = 0.0;
}

float PID::step(float xr, float x) 
{ 
    this->e2 = this->e1;
    this->e1 = this->e0;
    this->e0 = xr - x;

    float du = 0.0;
    
    du+= this->k0*this->e0;
    du+= this->k1*this->e1;
    du+= this->k2*this->e2;

    if (du > this->du_max)
    {
        du = du_max;
    }

    if (du < -this->du_max)
    {
        du = -du_max;
    }


    this->u = this->u + du;

    if (this->u > this->antiwindup)
    {
        this->u = this->antiwindup;
    }

    if (this->u < -this->antiwindup)
    {
        this->u = -this->antiwindup;
    }

    return this->u;
} 

void PID::reset()
{
    this->u = 0;
}
