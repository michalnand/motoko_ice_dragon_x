#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_

#include <drivers.h>
#include <stdint.h>
#include <array.h>


#define IR_SENSORS_COUNT        ((unsigned int)4)


/*
    IR left         :   PB1     ADC1, ADC2, IN9
    IR front left   :   PC5     ADC1, ADC2, IN15
    IR front right  :   PC4     ADC1, ADC2, IN14
    IR right        :   PB0     ADC1, ADC2, IN8
*/

class IRSensor
{
    public:
        void init();

        void callback();

        float obstacle_distance();
        int obstacle_detected();
            
    private:
        float calibration(float *callibration, float x);

    public:
        uint32_t measurement_id;

    private: 
        uint32_t state;
        Gpio<'B', 11, GPIO_MODE_OUT> ir_led; 

        Array<int, IR_SENSORS_COUNT> ir_off;
        Array<int, IR_SENSORS_COUNT> ir_on;

        float filter_coeff;
            
    public:
        Array<float, IR_SENSORS_COUNT> distance;

};

#endif