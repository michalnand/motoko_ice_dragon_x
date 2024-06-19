#include <identification.h>
#include <drivers.h>
#include <fmath.h>
#include <shaper.h>
#include <pid.h>


#define LED_GPIO        TGPIOE
#define LED_PIN         2



void motor_identification()
{
    //stop motor
    motor_control.halt();
    timer.delay_ms(200); 

    uint32_t n_steps = 500;

    //1, estimate motor constant k, on different input values

    float u_values[10] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

    float k_mean     = 0.0;
    float x_var_mean = 0.0;

    for (unsigned int j = 0; j < 10; j++)
    {
        //run motor with desired input and wait for steady state
        float u_in = u_values[j];

        motor_control.set_right_torque(u_in*MOTOR_CONTROL_MAX_TORQUE); 
        timer.delay_ms(500);    

        //estimate average velocity
        float x_mean = 0.0;
        for (unsigned int i = 0; i < n_steps; i++)
        {
            float x = motor_control.get_right_velocity();
            x_mean+= x;
            timer.delay_ms(1); 
        }
        x_mean = x_mean/n_steps;    

        //estimate average variance - encoder noise
        float x_var = 0.0;
        for (unsigned int i = 0; i < n_steps; i++)
        {
            float x = motor_control.get_right_velocity();
            x_var+= (x - x_mean)*(x - x_mean);
            timer.delay_ms(1);
        }
        x_var = x_var/n_steps;  

        float k = x_mean/u_in;
        k_mean+= x_mean/u_in;   
        x_var_mean+= x_var;

        terminal << "u_in   = " << u_in << "\n";
        terminal << "k      = " << k << "\n";
        terminal << "x_mean = " << x_mean << "\n";
        terminal << "x_var  = " << x_var << "\n";
        terminal << "\n\n";
    }   


    //print summary results
    k_mean     = k_mean/10.0;
    x_var_mean = x_var_mean/10.0;

    terminal << "k          = " << k_mean << "\n";
    terminal << "x_var_mean = " << x_var_mean << "\n";
    
    terminal << "\n\n";


    //2, estimate time constant by oscilating motor

    motor_control.halt();
    timer.delay_ms(200); 

    float u_in = 0.25;
    uint32_t periods = 0; 

    n_steps = 2000;

    for (unsigned int i = 0; i < n_steps; i++)
    {
        motor_control.set_right_torque(u_in*MOTOR_CONTROL_MAX_TORQUE);
        float x = motor_control.get_right_velocity()*60.0/(2.0*PI);

        if (u_in > 0.0) 
        {
            if (x > 0.632*k_mean*u_in)
            {
                u_in = -u_in;
                periods++;
            }
        }
        else
        {
            if (x < 0.632*k_mean*u_in)
            {
                u_in = -u_in;
                periods++;  
            }
        }

        timer.delay_ms(1);
    }
    
    motor_control.halt(); 
    timer.delay_ms(200);     
    
    float t_period = (2*n_steps/periods)/PI;
    terminal << "periods = " << periods << "\n";
    terminal << "tau     = " << t_period << "[ms]\n";
    terminal << "\n\n";
}






void robot_dynamics_identification()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

    motor_control.halt();
    timer.delay_ms(200);

    terminal << "\n\n\n";
    terminal << "starting identification\n";
    terminal << "\n\n\n";

    //dims in mm
    float wheel_diameter = 34.0;
    float wheel_brace    = 80.0;

    PID forward_pid, turn_pid;

    forward_pid.init(0.002, 0.0, 0.0, 1.0); 
    turn_pid.init(0.2, 0.0, 1.0, 1.0);  
 
    //input shaper
    Shaper forward_shaper, turn_shaper;
    
    //shaper init   
    forward_shaper.init(0.004, -0.004);    
    turn_shaper.init(0.1, -0.1);      

    float distances[]  = {0.0, 150.0, 0.0, -150.0, 0.0, 0.0,  0.0,  0.0};
    float angles[]     = {0.0, 0.0, 0.0,  0.0, 0.0, 90.0, 0.0, -90.0};

    //float distances[]  = {0.0, 100.0};
    //float angles[]     = {0.0, 90.0}; 


    float speed_max = MOTOR_CONTROL_MAX_VELOCITY;

    uint32_t steps = 0; 

    while (true)
    {
        int32_t time_start = timer.get_time();

        uint32_t idx = (steps/200)%8;
        
        float req_forward = distances[idx]; 
        float req_turn    = angles[idx]*PI/180.0;


        //obtain current state
        float left_position  = motor_control.get_left_position();
        float right_position = motor_control.get_right_position();

        float distance = 0.5*(right_position + left_position)*0.5*wheel_diameter;
        float angle    = 0.5*(right_position - left_position)*wheel_diameter / wheel_brace;

        // compute controller output
        float forward_curr = forward_pid.step(req_forward, distance);
        float turn_curr    = turn_pid.step(req_turn, angle);
      
        // compute shaped output
        float forward_curr_shaped = forward_shaper.step(forward_curr);
        float turn_curr_shaped    = turn_shaper.step(turn_curr);

        float u_left  = forward_curr_shaped - turn_curr_shaped;
        float u_right = forward_curr_shaped + turn_curr_shaped; 

        u_left  = clip(u_left, -1.0, 1.0);
        u_right = clip(u_right, -1.0, 1.0);

        // send to motors
        motor_control.set_left_velocity(u_left*speed_max);
        motor_control.set_right_velocity(u_right*speed_max);

        terminal << steps << " " << forward_curr << " " << turn_curr << " " << forward_curr_shaped << " " << turn_curr_shaped << " " << distance << " " <<  angle << " " << "\n";
        steps++;

        int32_t time_stop = timer.get_time();

        //compute delay time
        int32_t time_wait = 4 - (time_stop - time_start);
        if (time_wait < 0)
        {
            time_wait = 1;
        }

        timer.delay_ms(time_wait);
    }
}
