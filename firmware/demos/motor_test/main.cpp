#include <drivers.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    //float velocities[] = {100, 200, 300, 500};
    float velocities[] = {0, 400};

    uint32_t steps = 0;
    while (true)    
    {
        uint32_t idx = (steps/50)%2;    

        float req_velocity = velocities[idx];

        motor_control.set_left_velocity(req_velocity*(2.0*PI)/60.0);
        motor_control.set_right_velocity(req_velocity*(2.0*PI)/60.0);

        uint32_t n_samples = 50;    
        float left_u             = 0.0;
        float left_velocity      = 0.0;
        float left_velocity_hat  = 0.0;

        float right_u            = 0.0;
        float right_velocity     = 0.0;
        float right_velocity_hat = 0.0; 


        for (unsigned int n = 0; n < n_samples; n++)    
        {
            left_u+= motor_control.get_left_u();
            left_velocity+= motor_control.get_left_velocity()*60.0/(2.0*PI);
            left_velocity_hat+= motor_control.get_left_velocity_fil()*60.0/(2.0*PI);
            
            right_u+= motor_control.get_right_u();
            right_velocity+= motor_control.get_right_velocity()*60.0/(2.0*PI);
            right_velocity_hat+= motor_control.get_right_velocity_fil()*60.0/(2.0*PI);

            timer.delay_ms(1); 
        }

        left_u              = left_u/n_samples;
        left_velocity       = left_velocity/n_samples;
        left_velocity_hat   = left_velocity_hat/n_samples;

        right_u             = right_u/n_samples;  
        right_velocity      = right_velocity/n_samples;
        right_velocity_hat  = right_velocity_hat/n_samples;




        terminal << "### json\n";
        terminal << "{\"data\" : [ ";
        
        terminal << req_velocity << ", ";
        
        terminal << left_u << ", ";
        terminal << left_velocity << ", ";
        terminal << left_velocity_hat << ", ";

        terminal << right_u << ", ";
        terminal << right_velocity << ", ";
        terminal << right_velocity_hat << " ";
        terminal << "]}\n";
        terminal << "### end\n\n\n";

        steps++;
    }
    /*
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
    */

    return 0;
}
  