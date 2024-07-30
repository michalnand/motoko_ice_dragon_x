#include <drivers.h>
#include <pid.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    // constant forward speed
    float max_speed     = 0.2;
    float curr_speed    = 0.0;

    // controller parameters
    float kp = 0.2;
    float ki = 0.0;  
    float kd = 0.6;
    float antiwindup = 1.0;

    PID controller(kp, ki, kd, antiwindup);

    while (true)
    {
        // required value
        float xr = 0.0;

        // observed value
        float x  = line_sensor.left_position;
        
        // controller computes error internaly
        float u = controller.step(xr, x);

        // forward speed ramp, to avoid kick
        curr_speed = clip(curr_speed + 0.005, 0.0, max_speed);

        float u_right = curr_speed + u;
        float u_left  = curr_speed - u;

        // send turning command to motors
        // scale from -1, 1 into max velocity range
        motor_control.set_right_velocity(u_right*MOTOR_CONTROL_MAX_VELOCITY);
        motor_control.set_left_velocity(u_left*MOTOR_CONTROL_MAX_VELOCITY);

        // discrete dt delay
        timer.delay_ms(4);
    }

    return 0;
}
  