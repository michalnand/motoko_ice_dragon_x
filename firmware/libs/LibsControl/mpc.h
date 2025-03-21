#ifndef _MPC_H_
#define _MPC_H_

#include <stdint.h>
#include <matrix.h>


/*
MPC - analytical MPC
system_order    : system order, num of states
system_inputs   : num of controllable inputs into plat
*/
template<uint32_t system_order, uint32_t system_inputs, uint32_t prediction_horizon>
class MPC
{

    public:
        void init(const float *phi, const float *omega, const float *sigma, float antiwindup)
        {   
            this->x.init();
            this->xr.init();
            this->u.init();
            this->saturation.init();
              
            this->phi.from_array((float*)phi);
            this->omega.from_array((float*)omega);
            this->sigma.from_array((float*)sigma);

            this->antiwindup = antiwindup;
        }

        void set_xr(uint32_t step, uint32_t input, float  value)
        {
            uint32_t idx = step*system_order + input;
            this->xr[idx] = value;
        }

        void set_constant_xr(uint32_t input, float value)
        {   
            for (unsigned int i = 0; i < prediction_horizon; i++)
            {
                uint32_t idx = i*system_order + input;
                this->xr[idx] = value;
            }
        }

        void step() 
        { 
            // analytical mpc
            auto s  = this->xr - this->phi*x - this->omega*this->u;
            auto du = this->sigma*s;    
            
            // integration
            auto u_new  = this->u + du;

            // antiwindup
            this->u     = u_new.clip(-antiwindup, antiwindup);
            
            // set saturation flags is any
            this->_detect_saturation(u_new, this->u);
        } 

    private:
        void _detect_saturation(Matrix<float, system_inputs, 1> &u_raw, Matrix<float, system_inputs, 1> &u_sat, float eps = 0.0001)
        {
            for (unsigned int i = 0; i < system_inputs; i++)
            {
                float d = u_raw[i] - this->u[i];

                if (d > eps) 
                {
                    this->saturation[i] = 1;
                }
                else if (d < -eps)
                {   
                    this->saturation[i] = -1;
                }
                else
                {
                    this->saturation[i] = 0;
                }
            }
        }


    public:
        //inputs and outputs
        Matrix<float,   system_order, 1>                        x;
        Matrix<float,   system_order*prediction_horizon, 1>     xr;
        Matrix<float,   system_inputs, 1>                       u; 
        Matrix<int,     system_inputs, 1>                       saturation; 

    private:
        float antiwindup;

        //controller gain
        Matrix<float, system_order*prediction_horizon, system_order>  phi;
        Matrix<float, system_order*prediction_horizon, system_inputs> omega;
        Matrix<float, system_inputs, system_order*prediction_horizon> sigma;
};



#endif