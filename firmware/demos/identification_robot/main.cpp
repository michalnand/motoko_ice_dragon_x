#include <drivers.h>


#define WHEEL_BRACE             ((float)78.0)
#define WHEEL_DIAMETER          ((float)28.0)

#include <pid.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();


    float max_rpm = 1500;

    uint32_t samples    = 0;


    float req_distance[] = {0.0,  0.0,  0.0,   0.0,  0.0, 25.0,  0.0, -25.0};
    float req_angle[]    = {0.0, 40.0,  0.0, -40.0,  0.0,  0.0,  0.0,  0.0};

    

    float distance_prev = 0.0;
    float distance      = 0.0;
    float angle_prev    = 0.0;
    float angle         = 0.0;

    /*
    while (true)
    {
        //estimate current state 
        float right_position = motor_control.get_right_position_smooth();
        float left_position  = motor_control.get_left_position_smooth();

        distance_prev = distance;   
        distance      = 0.25*(right_position + left_position)*WHEEL_DIAMETER;

        angle_prev    = angle;
        angle         = 0.5*(right_position - left_position)*WHEEL_DIAMETER/WHEEL_BRACE;

        terminal << distance << " ";
        terminal << angle*(float)180.0/PI << "\n";

        timer.delay_ms(100);
    }
    */


    PID pid_forward(0.5, 0.01, 2.5, 1.0, 0.002);
    PID pid_turn(0.5, 0.01, 2.5, 1.0, 0.01);
        
    
    float dt = 0.0;

    while (true)
    {
        uint32_t time_start = timer.get_time();

        uint32_t idx = (samples/200)%8; 

        float distance_req  = req_distance[idx];
        float angle_req     = req_angle[idx]*PI/180.0;

        // estimate current state 
        float right_position = motor_control.get_right_position();
        float left_position  = motor_control.get_left_position();

        distance_prev = distance;   
        distance      = 0.25*(right_position + left_position)*WHEEL_DIAMETER;

        angle_prev    = angle;
        angle         = 0.5*(right_position - left_position)*WHEEL_DIAMETER/WHEEL_BRACE;


        float u_forward = pid_forward.step(distance_req, distance);
        float u_turn    = pid_turn.step(angle_req, angle);


        float u_right = u_forward + u_turn;
        float u_left  = u_forward - u_turn;

        motor_control.set_right_velocity(u_right*max_rpm*2.0*PI/60.0);
        motor_control.set_left_velocity(u_left*max_rpm*2.0*PI/60.0);

        terminal << dt << " " << u_forward << " " << u_turn << " " << distance << " " << angle << "\n";
        
        uint32_t time_stop = timer.get_time();  

        dt = 0.9*dt + 0.1*(time_stop - time_start);

        uint32_t dt_diff = clip(4.0 - dt, 0.0, 4.0);
        timer.delay_ms(dt_diff);  

        samples++;
    }


    motor_control.set_left_velocity(0.0);
    motor_control.set_right_velocity(0.0);


    while (true)      
    {
        timer.delay_ms(500);
    }

    return 0;
}
  