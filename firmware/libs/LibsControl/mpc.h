#ifndef _MPC_H_
#define _MPC_H_

#include <stdint.h>
#include <matrix.h>
#include <fmath.h>

/*
MPC - analytical MPC
system_order    : system order, num of states
system_inputs   : num of controllable inputs into plat
*/
template<uint32_t system_order, uint32_t system_inputs, uint32_t prediction_horizon>
class MPC
{

    public:
        void init(const float *phi, const float *omega, const float *sigma)
        {   
            this->x.init();
            this->xr.init();
            this->u.init();
            this->saturation.init();
              
            this->phi.from_array((float*)phi);
            this->omega.from_array((float*)omega);
            this->sigma.from_array((float*)sigma);
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

        /*
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
        */  

        void step() 
        { 
            // analytical mpc
            auto s      = this->xr - this->phi*x;
            auto u_new  = this->sigma*s;    
            
            // antiwindup
            float u_forward = u_new[0];
            float u_turn    = u_new[1];
            
            u_turn = clip(u_turn, -1.0f, 1.0f);
            
            // clamp u forward
            float max_u_forward = fmin(1.0 - u_turn, 1.0 + u_turn);
            float min_u_forward = fmax(-1.0 - u_turn, -1.0 + u_turn);
            
            u_forward = clip(u_forward, min_u_forward, max_u_forward);

            this->u[0] = u_forward;
            this->u[1] = u_turn;

            // set saturation flags is any
            this->_detect_saturation(u_new, this->u);
        } 


    private:
        void _detect_saturation(Matrix<float, system_inputs, 1> &u_raw, Matrix<float, system_inputs, 1> &u_sat, float eps = 0.0001)
        {
            for (unsigned int i = 0; i < system_inputs; i++)
            {
                float d = u_raw[i] - u_sat[i];

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
        //controller gain
        Matrix<float, system_order*prediction_horizon, system_order>  phi;
        Matrix<float, system_order*prediction_horizon, system_inputs> omega;
        Matrix<float, system_inputs, system_order*prediction_horizon> sigma;
};



#endif