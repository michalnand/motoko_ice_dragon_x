#ifndef _S_CURVE_H_
#define _S_CURVE_H_


class SCurve
{
    public:
        void init(float tau1, float tau2);
        void reset(float x_init = 0, float v_init = 0);
        float smooth_position(float x_req, float x_meas, float v_meas, float dt);

    private:
        float _x1, _x2;
    
        float tau1, tau2;
};

#endif