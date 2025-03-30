#include <drivers.h>

#include <path_planner.h>



int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";
    terminal << "system clock " << SystemCoreClock/1000000 << "MHz\n";

    // wait for key press
    int key_result = button();

    PathPlanner path_planner;

    path_planner.init();


    uint32_t steps = 0;

    
    /*
    // position control demo 
    
    while (true)    
    {                
        for (unsigned int i = 0; i < 10; i++)
        {
            path_planner.direct_control(0.0, 90.0*PI/180.0);
            timer.delay_ms(200); 
            path_planner.direct_control(0.0, 0.0);
            timer.delay_ms(200); 
        }

        timer.delay_ms(500);


        for (unsigned int i = 0; i < 10; i++)
        {
            path_planner.direct_control(0.0, 90*PI/180.0);
            timer.delay_ms(400); 
            path_planner.direct_control(0.0, 180*PI/180.0);
            timer.delay_ms(400); 
            path_planner.direct_control(0.0, 0.0);
            timer.delay_ms(400); 
        }

        timer.delay_ms(500);


        for (unsigned int i = 0; i < 10; i++)
        {
            path_planner.direct_control(100.0, 0.0);
            timer.delay_ms(500); 
            path_planner.direct_control(200.0, 0.0);
            timer.delay_ms(500); 
            path_planner.direct_control(0.0, 0.0);
            timer.delay_ms(500); 
        }

        timer.delay_ms(500);
    }
    */

    /*
    float distances[] = {0.0, 200.0};

    while (true)
    {
        uint32_t idx = (steps/500)%2;       
        
        float d_req = distances[idx];

        path_planner.set_position(d_req, 0.0);

        timer.delay_ms(4); 

        steps++;
    }
    */


    float angles[] = {0.0, 90.0};

    while (true)
    {
        uint32_t idx = (steps/500)%2;       
        
        float a_req = angles[idx]*PI/180.0;

        path_planner.set_position(0.0, a_req);

        timer.delay_ms(4); 

        steps++;
    }

    

    return 0;
}
  