#include <drivers.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    float velocities[] = {100, 200, 300, 500};

   
    while (true) 
    {
        for (unsigned int n = 0; n < 4; n++)
        {
            float req_velocity = velocities[n];

            motor_control.set_left_velocity(req_velocity*PI/180.0);
            motor_control.set_right_velocity(req_velocity*PI/180.0);

            timer.delay_ms(1500);

            float left_velocity      = 0.0;
            float left_velocity_hat  = 0.0;
            float right_velocity     = 0.0;
            float right_velocity_hat = 0.0;


            for (unsigned int n = 0; n < 64; n++)
            {
                left_velocity+= motor_control.get_left_velocity()*180.0/PI;
                left_velocity_hat+= motor_control.get_left_velocity_fil()*180.0/PI;
                right_velocity+= motor_control.get_right_velocity()*180.0/PI;
                right_velocity_hat+= motor_control.get_right_velocity_fil()*180.0/PI;

                timer.delay_ms(10); 
            }

            left_velocity       = left_velocity/64.0;
            left_velocity_hat   = left_velocity_hat/64.0;
            right_velocity      = right_velocity/64.0;
            right_velocity_hat  = right_velocity_hat/64.0;



            terminal << "required velocity " << req_velocity << " rpm\n";
            terminal << "left velocity     " << left_velocity << " " << left_velocity_hat << " rpm\n";
            terminal << "right velocity    " << right_velocity << " " << right_velocity_hat << " rpm\n";
            terminal << "\n\n";     

            timer.delay_ms(100); 
        }
    }

    return 0;
}
  