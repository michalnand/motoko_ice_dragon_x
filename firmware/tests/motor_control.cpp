#include <libs_drivers.h>
#include <tmath.h>

void motor_control_test()
{
    //stop motor
    float left_velocity_req  = 500*2.0*PI/60.0;
    float right_velocity_req = 500*2.0*PI/60.0;
    
    
    while (1)
    {
        motor_control.set_left_velocity(left_velocity_req);
        motor_control.set_right_velocity(right_velocity_req);

        int32_t left_rpm  = (motor_control.get_left_velocity()*60.0)/(2.0*PI);
        int32_t right_rpm  = (motor_control.get_right_velocity()*60.0)/(2.0*PI);

        terminal << "rpm    = " << right_rpm << " " << left_rpm << "\n";

        timer.delay_ms(100);
    }
}
