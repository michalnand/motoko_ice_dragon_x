#ifndef _MotionKalman_H_
#define _MotionKalman_H_


class MotionKalman
{
    public:
        void init(float k0, float k1, float dt);
        void step(float position_measurement);

    private:
        float k0, k1, dt;

    public:
        float position_hat, velocity_hat;
};


class MotionFilterEMA
{
    public:
        void init(float k0, float k1);
        void step(float position_measurement);

    private:
        float k0, k1;

    public:
        float position_hat, velocity_hat;
};



#endif

