#ifndef _LQG_H_
#define _LQG_H_

#include <stdint.h>
#include <matrix.h>


/*
LQG - linear quadratic gaussian control with Kalman observer
system_order    : system order, num of states
system_inputs   : num of controllable inputs into plat
*/
template<uint32_t system_order, uint32_t system_inputs>
class LQG
{
    public:
        void init(float *a, float *b,float *k, float *ki, float *f, float antiwindup)
        {   
            this->x.init();
            this->xr.init();
            this->u.init();
              
            this->a.from_array(a);
            this->b.from_array(b);

            this->k.from_array(k);
            this->ki.from_array(ki);
            this->f.from_array(f);

            this->integral_action.init();
            this->x_hat.init();

            this->antiwindup = antiwindup;
        }

        void step()
        { 
            // integral action  
            auto error = this->xr - this->x;
            auto integral_action_new = this->integral_action + this->ki*error;
            
            //LQR controll law  
            auto u_new = this->k*this->x_hat*(-1.0) + integral_action_new;

            //antiwindup with conditional integration
            this->u = u_new.clip(-antiwindup, antiwindup);
            this->integral_action = integral_action_new - (u_new - this->u);
            
            // kalman observer
            // only y is known, and using knowledge of dynamics, 
            // the full state x_hat can be reconstructed
            auto prediction_error = this->x - this->x_hat;  
            this->x_hat = this->a*this->x_hat + this->b*this->u + this->f*prediction_error;
        }       

    
    public:
        //inputs and outputs
        Matrix<float, system_order, 1> x;
        Matrix<float, system_order, 1> xr;
        Matrix<float, system_inputs, 1>  u;

    private:
        float antiwindup;

        //plant dynamics
        Matrix<float, system_order, system_order>   a;
        Matrix<float, system_order, system_inputs>  b; 

        //kalman gain
        Matrix<float, system_order, system_order> f; 
        
        //controller gain
        Matrix<float, system_inputs, system_order>      k;
        Matrix<float, system_inputs, system_order>    ki;

    public:
        //internal state
        Matrix<float, system_inputs, 1> integral_action;
        Matrix<float, system_order, 1>  x_hat;
};


#endif