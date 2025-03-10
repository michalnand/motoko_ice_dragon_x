#include <drivers.h>

#include <position_control.h>


PositionControl position_control;

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";
    terminal << "system clock " << SystemCoreClock/1000000 << "MHz\n";

    // wait for key press
    int key_result = button();


    position_control.init();

    /*
    while (true)
    {
        position_control.steps = 0;
        motor_control.steps = 0;

        timer.delay_ms(1000);

        volatile uint32_t motor_control_steps    = motor_control.steps;
        volatile uint32_t position_control_steps = position_control.steps;

        terminal << timer.get_time() << " " << motor_control_steps << " " << position_control_steps << "\n";
    }   
    */

    /*
    while (true)
    {
        float x = position_control.get_distance();
        float a = position_control.get_angle();

        float v = position_control.get_velocity();
        float w = position_control.get_angular_velocity();

      
        terminal << x << " " << a << " " << v << " " << w << " " << "\n";

        timer.delay_ms(50); 
    }
    */

 

    
    //float distances[] = {0.0, 60.0};
    //float angles[]    = {0.0, 0.0};

    float distances[] = {0.0, 0.0};
    float angles[]    = {0.0, 90.0};

    uint32_t steps = 0;         
    while (true)        
    {
        uint32_t idx = (steps/50)%2;    

        float req_distance  = distances[idx];
        float req_angle     = angles[idx]*PI/180.0;  

        float distance = position_control.get_distance();
        float angle    = position_control.get_angle()*180.0/PI;


        position_control.set_single_point(req_distance, req_angle);

        terminal << "### json\n";
        terminal << "{\"data\" : [ ";   
        
        terminal << req_distance << ", ";
        
        terminal << req_angle << ", ";
        terminal << distance << ", ";
        terminal << angle << " ";

        terminal << "]}\n";
        terminal << "### end\n\n\n";

        steps++;

        timer.delay_ms(10);
    }

    return 0;
}
  