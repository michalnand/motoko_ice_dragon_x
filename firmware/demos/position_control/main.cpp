#include <drivers.h>

#include <position_control.h>


PositionControl position_control;

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();


    position_control.init();

    
    float distances[] = {0.0, 60.0};
    float angles[]    = {0.0, 0.0};

    //float distances[] = {0.0, 0.0};
    //float angles[]    = {0.0, 90.0};

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
  