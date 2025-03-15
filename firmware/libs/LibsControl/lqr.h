#ifndef _LQR_H_
#define _LQR_H_

#include <stdint.h>
#include <matrix.h>


/*
LQR - linear quadratic regulator
system_order    : system order, num of states
system_inputs   : num of controllable inputs into plat
*/
template<uint32_t system_order, uint32_t system_inputs>
class LQR
{

    public:
        void init(float *k, float *ki, float antiwindup)
        {   
            this->x.init();
            this->xr.init();
            this->u.init();
            this->saturation.init();
              
            this->k.from_array(k);
            this->ki.from_array(ki);

            this->integral_action.init();

            this->antiwindup = antiwindup;
        }

        void step(bool is_saturated = false) 
        { 
            // integral action  
            auto error = this->xr - this->x; 

            //LQR controll law  
            auto u_new = this->k*error ;

            //antiwindup with back calculating integration
            this->u = u_new.clip(-antiwindup, antiwindup);

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
                integral_action[idx] = 0.0;
            }
            else
            {
                integral_action.init(0.0);
            }
        }



    public:
        //inputs and outputs
        Matrix<float, system_order, 1> x;
        Matrix<float, system_order, 1> xr;
        Matrix<float, system_inputs, 1>  u; 
        Matrix<int, system_inputs, 1>  saturation; 

    private:
        float antiwindup;

        //controller gain
        Matrix<float, system_inputs, system_order> k;
        Matrix<float, system_inputs, system_order> ki;

    public:
        //internal state
        Matrix<float, system_inputs, 1> integral_action;
};


#endif