#include <drivers.h>

#include <path_planner.h>

void motion_test(PathPlanner &path_planner)
{
    float turn_search_distance      = 50.0;
    float forward_search_distance   = 80.0;
    float r_search  = 90.0;
    float r_max     = 10000.0;

    float speed     = 500.0;
    float acc_max   = 3.0*9.81*1000.0;

    int state       = 2;
    int way         = 1;    

    
    while (true)
    {
        // left or right line searching
        if (state == 0 || state == 1)
        {
            //turn until line found, or distance trehold
            float start_distance      = path_planner.position_control.get_distance();
            float target_distance     = start_distance + turn_search_distance;

            while (path_planner.position_control.get_distance() < target_distance)
            { 
                path_planner.set_circle_motion(way*r_search, 0.25*speed, acc_max);
                timer.delay_ms(4);      

                if (line_sensor.line_lost_type == LINE_LOST_NONE)
                {
                    //return;
                } 
            }       

            while (path_planner.position_control.get_distance() > start_distance)
            { 
                path_planner.set_circle_motion(way*r_search, -0.25*speed, acc_max);
                timer.delay_ms(4);    
                
                if (line_sensor.line_lost_type == LINE_LOST_NONE)
                {
                    //return;
                }
            }     

            way*= -1;

            state++;
        }
        // line lost in midle, center
        // go forward, until line found or maximal distance reached
        else
        {          
            float start_distance      = path_planner.position_control.get_distance();
            float target_distance     = start_distance + forward_search_distance;

            while (path_planner.position_control.get_distance() < target_distance)
            {   
                path_planner.set_circle_motion(r_max, speed, acc_max);
                timer.delay_ms(4);  

                if (line_sensor.line_lost_type == LINE_LOST_NONE)
                {
                    //return; 
                }
            }     

            state = 0;  
        }
    }
}

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


    while (1)
    {
        motion_test(path_planner);
    }

    

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


    float r_req[] = {80.0, 100.0, 200.0};
    float v_req[] = {200.0, 500.0, 800.0};
    
    while (true)    
    {
        uint32_t idx = (steps/500)%3;  
        float v     = path_planner.position_control.get_velocity();        

        path_planner.set_circle_motion(r_req[idx], v_req[idx], 3.0*9.81*1000); 

        terminal << v_req[idx] << " " << v << "\n";
        
        steps++;
        timer.delay_ms(4); 
    }
    
    
    float distances[] = {0.0, 100.0, 0.0, 200.0, 0.0, 500.0};

    while (true)    
    {
        uint32_t idx = (steps/500)%6;       
        
        float d_req = distances[idx];

        path_planner.set_position(d_req, 0.0);

        timer.delay_ms(4); 

        steps++;
    }
    

    
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
  