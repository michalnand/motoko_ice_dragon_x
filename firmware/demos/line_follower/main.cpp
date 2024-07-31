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
    float max_speed     = 0.4;

    // controller parameters
    /*
    float kp = 0.25; 
    float ki = 0.0001;         
    float kd = 0.5;   
    */      

    float kp = 0.5;         
    float ki = 0.01;                
    float kd = 2.5;          

    float antiwindup = 1.0;  

    PID controller(kp, ki, kd, antiwindup);

    uint32_t steps = 0;
    while (true) 
    {
        // required value
        float xr = 0.0;

        // observed value
        float x  = line_sensor.left_position;
        
        // controller computes error internaly
        float turn = controller.step(xr, x);          

        float curr_speed = clip(max_speed - abs(turn), 0.0, max_speed);

        float u_right = curr_speed + turn;
        float u_left  = curr_speed - turn;

        u_right = clip(u_right, 0.0, 1.0);
        u_left  = clip(u_left, 0.0, 1.0);

        // send turning command to motors
        // scale from -1, 1 into max velocity range
        //motor_control.set_right_velocity(u_right*MOTOR_CONTROL_MAX_VELOCITY);
        //motor_control.set_left_velocity(u_left*MOTOR_CONTROL_MAX_VELOCITY);

        motor_control.set_right_torque(u_right);
        motor_control.set_left_torque(u_left);  

        // discrete dt delay
        timer.delay_ms(4);

        if ((steps%100) == 0)
        {
            terminal << x << "\n";
            terminal << turn << " " << curr_speed << "\n";
            terminal << u_left << " " << u_right << "\n";
            terminal << "\n\n";
        }

        steps++;
    }

    return 0;
}
  