#include <drivers.h>
#include <motor_identification.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    //motor_identification();

    
    float velocity_min = 0.0;
    float velocity_max = 1500.0;

    unsigned int n_steps;
    while (true)    
    {
        n_steps = 2000;    

        for (unsigned int i = 0; i < n_steps; i++)
        {
            float k = (1.0*i)/(n_steps-1);        
            float v = (1.0 - k)*velocity_min + k*velocity_max;
            motor_control.set_right_velocity(v*(2.0*PI/60.0));
            timer.delay_ms(1);
        }        

        timer.delay_ms(2000);


        for (unsigned int i = 0; i < n_steps; i++)
        {
            float k = (1.0*i)/(n_steps-1);
            float v = (1.0 - k)*velocity_max + k*velocity_min;
            motor_control.set_right_velocity(v*(2.0*PI/60.0));
            timer.delay_ms(1);
        }

        timer.delay_ms(800);


        motor_control.set_right_velocity(velocity_max);
        timer.delay_ms(2000);
        motor_control.set_right_velocity(velocity_min);
        timer.delay_ms(800);
    }

    motor_control.set_right_velocity(300*(2.0*PI/60.0));
    

    while (true)      
    {
        float position = motor_control.get_right_position()*180.0/PI;
        float rpm      = motor_control.get_right_velocity()*60/(2.0*PI);    
        terminal << position << " " << rpm << " " << "\n";
        timer.delay_ms(500);
    }

    return 0;
}
  