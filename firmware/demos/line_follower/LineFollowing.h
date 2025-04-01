#ifndef _LINE_FOLLOWING_H_
#define _LINE_FOLLOWING_H_

#include <drivers.h>
#include <path_planner.h>
#include "QEstimator.h" 

class LineFollowing
{
    public:
        void init();
        int main();

    private:
        float estimate_turn_radius(float sensor_reading, float eps);
        void line_search(uint32_t line_lost_type);
        void obstacle_avoid();

    private:
        PathPlanner path_planner;
        QEstimator<32> q_estimator;

    private:
        float speed_min, speed_max;
        float r_min,  r_max; 
        float q_penalty, qr_max, qr_min;
};


#endif