#include <drivers.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    Gpio<TGPIOB, 2, GPIO_MODE_OUT> led;   
    led = 1;

    while (true)
    {
        led = 1; 

        // convert wheel angle from radians to degrees and print
        terminal << "encoder sensor\n";
        terminal << "right " << motor_control.get_right_encoder() << " " << motor_control.get_right_position()*(float)(180.0/PI) << " " << motor_control.get_right_velocity()*(float)(180.0/PI) << "\n";
        terminal << "left  " << motor_control.get_left_encoder() << " " <<  motor_control.get_left_position()*(float)(180.0/PI) << " " << motor_control.get_left_velocity()*(float)(180.0/PI) << "\n";
        terminal << "\n\n";

        terminal << "\n\n";

        led = 0;
        timer.delay_ms(200);
    }

    return 0;
}
  