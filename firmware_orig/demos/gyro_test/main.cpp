#include <drivers.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    PID pid;  
    pid.init(50.0, 0.0, 200.0, 2.0*PI*1000.0/60.0);

    float angle = 0.0; 

     
    while (1)     
    {
        float angular_rate = gyro_sensor.read()*4.0/1000.0;

        angle+= angular_rate;
        timer.delay_ms(4);

        float u = pid.step(0.0, angle);

        motor_control.set_left_velocity(-u);
        motor_control.set_right_velocity(u);
    }


    return 0;
}
  