#ifndef _LINE_FOLLOWING_H_
#define _LINE_FOLLOWING_H_

#include <drivers.h>
#include <path_planner.h>
#include <filter.h> 

class LineFollowing
{
    public:
        void init();
        int main();

    private:
        float estimate_turn_radius(float sensor_reading, float eps);

    private:
        PathPlanner path_planner;
        FirFilter<float, 32> quality_filter;

    private:
        float speed_min, speed_max;
        float r_min,  r_max; 
        float q_penalty, qr_max, qr_min;
};


#endif