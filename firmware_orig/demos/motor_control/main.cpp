#include <drivers.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    while (true)        
    {   
        float speed_rpm = 500;
        motor_control.set_left_velocity(speed_rpm*(2.0*PI)/60.0);
        motor_control.set_right_velocity(speed_rpm*(2.0*PI)/60.0);
        timer.delay_ms(1000);   

        float left_rpm  =  motor_control.get_left_velocity_smooth()*60.0/(2.0*PI);
        float right_rpm =  motor_control.get_right_velocity_smooth()*60.0/(2.0*PI);
        
        terminal << left_rpm << " " << right_rpm << "\n";
    }


    float speed_max = 2000;
    float speed     = 0;
    

    while (true)        
    {   
        motor_control.set_left_velocity(10*(2.0*PI)/60.0);
        motor_control.set_right_velocity(10*(2.0*PI)/60.0);
        timer.delay_ms(5000);  
        
        motor_control.set_left_velocity(60*(2.0*PI)/60.0);
        motor_control.set_right_velocity(60*(2.0*PI)/60.0);
        timer.delay_ms(5000);   

        motor_control.set_left_velocity(0*(2.0*PI)/60.0);
        motor_control.set_right_velocity(0*(2.0*PI)/60.0);
        timer.delay_ms(500);

        for (float speed = 0; speed < speed_max; speed+=1.0)
        {
            motor_control.set_left_velocity(speed*(2.0*PI)/60.0);
            motor_control.set_right_velocity(speed*(2.0*PI)/60.0);
            timer.delay_ms(1);
        }   

        timer.delay_ms(1000);

        motor_control.set_left_velocity(0*(2.0*PI)/60.0);
        motor_control.set_right_velocity(0*(2.0*PI)/60.0);

        timer.delay_ms(500);    



        for (float speed = 0; speed < speed_max; speed+=10)
        {
            motor_control.set_left_velocity(speed*(2.0*PI)/60.0);
            motor_control.set_right_velocity(speed*(2.0*PI)/60.0);
            timer.delay_ms(1);
        }

        timer.delay_ms(1000);

        motor_control.set_left_velocity(0*(2.0*PI)/60.0);
        motor_control.set_right_velocity(0*(2.0*PI)/60.0);

        timer.delay_ms(500);
    }

      
   


    /*
    float velocities[] = {0, 10, 50, 100, 400, 1000};   

    uint32_t steps = 0;           
    while (true)    
    {
        uint32_t idx = (steps/50)%6;    

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
    */

    return 0;
}
  