#ifndef _PID_H_
#define _PID_H_


/*
    discrete PID controller
*/
class PID
{
    public:
        PID();
        PID(float kp, float ki, float kd, float antiwindup);
        
        void init(float kp, float ki, float kd, float antiwindup);
        float step(float xr, float x);
        void reset();

        float get_x_hat();
      
    private:
        float k0, k1, k2;
        float e0, e1, e2;
        float antiwindup;

    public:
        float u;

};


#endif