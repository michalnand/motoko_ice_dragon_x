#ifndef _PWM_H_
#define _PWM_H_

//20kHz PWM, 216MHz/2 is timer clock source
#define PWM_FREQUENCY           ((uint32_t)20000)
#define PWM_PERIOD              ((uint32_t)(216000000/2)/PWM_FREQUENCY - 1)



class PWMLeft 
{   
    public:
        void init();
        void set(int32_t pwm);
};

class PWMRight
{   
    public:
        void init();
        void set(int32_t pwm);
};

#endif