#include <libs_drivers.h>
#include <tmath.h>

// run both BLDC motors forward, and backward, using FOC controll loop
void motor_driver_test()
{
    while (1)   
    {
        motor_control.set_left_torque(0.8); 
        motor_control.set_right_torque(0.8);
        timer.delay_ms(1000);
        
        motor_control.set_left_torque(-0.8);
        motor_control.set_right_torque(-0.8);
        timer.delay_ms(1000);

        motor_control.set_left_torque(0.0);
        motor_control.set_right_torque(0.0);
        timer.delay_ms(1000);
    }
}
