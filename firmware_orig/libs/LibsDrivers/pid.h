#ifndef _PID_H_
#define _PID_H_


/*
    discrete PID controller
*/
class PID
{
    public:
        PID();
        PID(float kp, float ki, float kd, float antiwindup, float du_max);
        
        void init(float kp, float ki, float kd, float antiwindup, float du_max);
        float step(float xr, float x);
        void reset();

        float get_x_hat();
      
    private:
        float k0, k1, k2;
        float e0, e1, e2;
        float antiwindup, du_max;

    public:
        float u;

};


#endif