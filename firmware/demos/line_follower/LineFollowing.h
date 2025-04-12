#ifndef _LINE_FOLLOWING_H_
#define _LINE_FOLLOWING_H_

#include <drivers.h>
#include <path_planner.h>
#include "QEstimator.h" 
#include "split_line_detector.h"

class LineFollowing
{
    public:
        void init();
        int main();

    public:
        float estimate_turn_radius(float sensor_reading, float eps);
        void line_search(uint32_t line_lost_type, float curvature);
        void obstacle_avoid();
        void curtain_avoid();

        void led_blink(uint32_t count = 20);


    private:
        PathPlanner path_planner;
        QEstimator<32> q_estimator;
        SplitLineDetector split_line_detector;


    private:
        float speed_min, speed_max;
        float r_min,  r_max; 
        float q_penalty, qr_max, qr_min;

    private:
        uint32_t steps;
        Gpio<TGPIOB, 2, GPIO_MODE_OUT> led;   
        

};


#endif