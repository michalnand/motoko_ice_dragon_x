#ifndef _MPC_H_
#define _MPC_H_

#include <stdint.h>
#include <matrix.h>


/*
    MPC - unconstrained model predictive control
    system_order    : system order, num of states
    system_inputs   : num of controllable inputs into plat
    prediction_horizon : num of prediction steps
*/
template<uint32_t system_order, uint32_t system_inputs, uint32_t prediction_horizon>
class MPC
{

    public:
        void init(float *phi, float *omega, float *sigma, float antiwindup)
        {   
            this->x.init();
            this->xr.init();
            this->u.init();
              
            this->phi.from_array(phi);
            this->omega.from_array(omega);
            this->sigma.from_array(sigma);

            this->antiwindup = antiwindup;
        }

        void step()
        {
            auto s  = this->xr - this->phi@this->x - this->omega@this->u;
            auto du = this->sigma@s;     
            
            this->u = this->u + du;
            this->u = u_new.clip(-antiwindup, antiwindup);
        }

        // sets entire required trajectory 
        // with constant tiled vectors xr, xr.shape = (system_order, 1)
        void set_constant_xr(float *xr)
        {
            uint32_t ptr = 0;
            for (unsigned int n = 0; n < prediction_horizon; n++)
            {
                for (unsigned int i = 0; i < system_order; i++)
                {
                    this->xr[ptr] = xr[i];
                    ptr++;
                }
            }
        }

    public:
        //inputs and outputs
        //current system state
        Matrix<float, system_outputs, 1> x;

        //required trajectory
        Matrix<float, system_outputs*prediction_horizon, 1> xr;

        //controller output
        Matrix<float, system_inputs, 1>  u;

    private:    
        //controller matrices 
        Matrix<float, system_order*prediction_horizon, system_order>  phi;
        Matrix<float, system_order*prediction_horizon, system_inputs> omega;
        Matrix<float, system_inputs, system_order*prediction_horizon> sigma;

    
    private:
        float antiwindup;
};


#endif