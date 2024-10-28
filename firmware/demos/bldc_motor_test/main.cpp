#include <drivers.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    terminal << "starting\n";


    float torque[] = {0.0, 0.05, 0.1, 0.2, 0.5, 1.0};

   

    uint32_t steps = 0; 
    while (true)    
    {
        uint32_t idx = (steps/100)%6;    

        motor_control.set_left_torque(-torque[idx]);
        motor_control.set_right_torque(torque[idx]);


        steps++;

        timer.delay_ms(20);

        terminal << motor_control.get_left_encoder() << " " << motor_control.get_right_encoder() << "\n";
    }
   

    return 0;
}
  