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
    //float distances[] = {0.0, 60.0};
    //float angles[]    = {0.0, 0.0};

    float distances[] = {0.0, 0.0};
    float angles[]    = {0.0, 90.0};

    float dt = 1.0/250.0;   

    float angle_req = 0.0;
    while (true)    
    {
        uint32_t idx = (steps/20)%2;    
        
        float distance  = distances[idx];
        float angle     = angles[idx]*PI/180.0;
                
        //path_planner.point_following(distance, angle);
        path_planner.direct_control(distance, angle);

        timer.delay_ms(50); 
        
        steps++;
    }
    */

    /*
    // position control demo 
    float angle = 0.0;
    while (true)    
    {                
        for (unsigned int i = 0; i < 10; i++)
        {
            path_planner.direct_control(0.0, 90.0*PI/180.0);
            timer.delay_ms(100); 
            path_planner.direct_control(0.0, 0.0);
            timer.delay_ms(100); 
        }

        timer.delay_ms(300);


        for (unsigned int i = 0; i < 10; i++)
        {
            path_planner.direct_control(0.0, 90*PI/180.0);
            timer.delay_ms(150); 
            path_planner.direct_control(0.0, 180*PI/180.0);
            timer.delay_ms(150); 
            path_planner.direct_control(0.0, 0.0);
            timer.delay_ms(150); 
        }

        timer.delay_ms(300);
    }
    */

    

    float velocities[] = {0.0, 2000.0, -2000.0, 0.0};

    while (true)
    {
        uint32_t idx = (steps/1000)%4;       
        
        float v_req = velocities[idx];

        path_planner.line_following(v_req, 0.0);

        if ((steps%50) == 0)
        {
            terminal << v_req << " " << path_planner.position_control.get_velocity() << "\n";
        }   

        timer.delay_ms(4); 

        steps++;
    }
    

    //float distances[] = {0.0, 60.0, 200.0, 500.0};
    //float angles[]    = {0.0, 0.0,  0.0,   0.0};

    float distances[] = {0.0, 200.0, 500.0, 0.0};
    float angles[]    = {0.0, 0.0, 0.0, 0.0};


    while (true)
    {
        uint32_t idx = (steps/400)%4;      

        float d_req = distances[idx];
        float a_req = angles[idx];  

        path_planner.point_following(d_req, a_req*PI/180.0f);


        terminal << d_req << " " << path_planner.position_control.get_distance() << "\n";

        timer.delay_ms(4); 

        steps++;
    }

    return 0;
}
  