#ifndef _MotionKalman_H_
#define _MotionKalman_H_


class MotionKalman
{
    public:
        void init(float k0, float k1, float dt);
        void step(float x);

    private:
        float k0, k1, dt;
        float x0, x1;

    public:
        float position_hat, velocity_hat;
};


#endif

