#include <drivers.h>
#include <motor_identification.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    motor_identification();


    while (true)      
    {
        float position = motor_control.get_right_position()*180.0/PI;
        float rpm      = motor_control.get_right_velocity()*60/(2.0*PI);    
        terminal << position << " " << rpm << " " << "\n";
        timer.delay_ms(500);
    }

    return 0;
}
  