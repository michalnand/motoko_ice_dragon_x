#ifndef _LINE_SENSOR_H_
#define _LINE_SENSOR_H_

#include <drivers.h>
#include <stdint.h>
#include <array.h>

 

#define LINE_SENSOR_STEP                ((int32_t)128)

//sensitivity
//#define LINE_SENSOR_THRESHOLD           ((int32_t)500)   
//#define LINE_SENSOR_THRESHOLD           ((int32_t)300)   
#define LINE_SENSOR_THRESHOLD           ((int32_t)250)   
//#define LINE_SENSOR_THRESHOLD           ((int32_t)200)   
//#define LINE_SENSOR_THRESHOLD           ((int32_t)150)   

    

//brace from first to last sesor in mm
#define SENSORS_BRACE                  ((float)70.0)

//sensors distance from wheel axis in mm
#define SENSORS_DISTANCE                  ((float)48.0)
    

#define LINE_SENSOR_COUNT   ((unsigned int)8)

//state where line was lost
#define LINE_LOST_NONE      ((unsigned char)0)
#define LINE_LOST_CENTER    ((unsigned char)1)
#define LINE_LOST_RIGHT     ((unsigned char)2)
#define LINE_LOST_LEFT      ((unsigned char)3)


 
class LineSensor
{
    protected:  
        Gpio<'C', 4, GPIO_MODE_OUT> sensor_led;        //sensor white led

    public:
        Array<int, LINE_SENSOR_COUNT> adc_result;

    public:
        //sensors normalisation, calibration constants
        Array<int, LINE_SENSOR_COUNT> weights;
        Array<int, LINE_SENSOR_COUNT> adc_calibration_q;
        Array<int, LINE_SENSOR_COUNT> adc_calibration_k;

    private:    
        //line sensors  
        Gpio<'A', 0, GPIO_MODE_AN> sensor_in_0;
        Gpio<'A', 1, GPIO_MODE_AN> sensor_in_1;
        Gpio<'A', 2, GPIO_MODE_AN> sensor_in_2;
        Gpio<'A', 3, GPIO_MODE_AN> sensor_in_3;
        Gpio<'A', 4, GPIO_MODE_AN> sensor_in_4;
        Gpio<'A', 5, GPIO_MODE_AN> sensor_in_5;
        Gpio<'A', 6, GPIO_MODE_AN> sensor_in_6;
        Gpio<'A', 7, GPIO_MODE_AN> sensor_in_7;

    public:
        uint32_t line_lost_type;
        uint32_t on_line_count;


        //this stores last valid line position <-1, 1>
        float left_position, right_position, center_position;

        float minimal_position, extremal_position;

        //raw line position into angle (radians)
        float left_angle, right_angle;


        uint32_t measurement_id;

    private:
        float angle_prev[4];

    public: 
        LineSensor();
        virtual ~LineSensor();

        void init();

        void callback(); 

    protected:
        void    process();
        int     integrate(int center_idx);

};

#endif
