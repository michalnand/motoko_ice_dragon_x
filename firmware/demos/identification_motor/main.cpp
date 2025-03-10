#include <drivers.h>
#include <motor_identification.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();


    motor_control.set_left_torque(0.2); 
    timer.delay_ms(500);      

    float k      = 0.05;
    float x_mean = 0.0;
    float x_var  = 0.0; 

    while (true)        
    {
        float x = motor_control.get_left_velocity();

        x_mean = (1.0 - k)*x_mean + k*x;
        x_var  = (1.0 - k)*x_var  + k*(x - x_mean)*(x - x_mean);

        float rpm = x_mean*60.0/(2.0*PI);

        terminal << " " << rpm << " " << x_mean << " " << x_var << " " << x << " " << "\n";

        timer.delay_ms(20);
    }
    

    motor_identification();


    while (true)      
    {
        timer.delay_ms(500);
    }

    return 0;
}
  