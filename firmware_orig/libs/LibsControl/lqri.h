#ifndef _LQRI_H_
#define _LQRI_H_

#include <stdint.h>
#include <matrix.h>

/*
LQR - linear quadratic regulator with delta u output
system_order    : system order, num of states
system_inputs   : num of controllable inputs into plat
*/
template<uint32_t system_order, uint32_t system_inputs>
class LQRI
{

    public:
        void init(float *k, float *ku, float du_max, float u_max)
        {   
            this->x.init();
            this->xr.init();
            this->u.init(); 
            this->saturation.init();
              
            this->k.from_array(k);
            this->ku.from_array(ku);

            this->du_max = du_max;
            this->u_max  = u_max;
        }

        void step(bool is_saturated = false) 
        { 
            // integral action  
            auto error = this->xr - this->x; 

            //LQR controll law  
            auto du = this->k*error - this->ku*this->u;

            //antiwindup with, clipping both du and u value
            du  = du.clip(-du_max, du_max);
            auto u_new  = this->u + du;
            this->u     = u_new.clip(-u_max, u_max);    

            for (unsigned int i = 0; i < system_inputs; i++)
            {
                float d = u_new[i] - this->u[i];

                if (d > 0.0001) 
                {
                    this->saturation[i] = 1;
                }
                else if (d < -0.0001)
                {   
                    this->saturation[i] = -1;
                }
                else
                {
                    this->saturation[i] = 0;
                }
            }
        } 

        void reset_integral_action(int idx = -1)
        {
            if (idx >= 0)
            {   
                this->u[idx] = 0.0;
            }
            else    
            {
                this->u.init(0.0);
            }
        }



    public:
        //inputs and outputs
        Matrix<float, system_order, 1>  x;
        Matrix<float, system_order, 1>  xr;
        Matrix<float, system_inputs, 1>  u; 
        Matrix<int, system_inputs, 1>  saturation; 

    private:
        //controller gain
        Matrix<float, system_inputs, system_order> k;
        Matrix<float, system_inputs, system_inputs> ku;

        float du_max, u_max;
};


#endif