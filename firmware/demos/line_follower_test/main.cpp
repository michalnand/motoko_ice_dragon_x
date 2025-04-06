#include <drivers.h>

#include <path_planner.h>


float estimate_turn_radius(float sensor_reading, float eps)
{
  float x = SENSORS_DISTANCE;
  float y = 0.5*SENSORS_BRACE*abs(sensor_reading);

  float r = (y*y + x*x)/(2.0*y + eps);

  return r;
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

    

    float velocity = 400.0;     

    float r_min = 80.0;
    float r_max = 10000.0;    

    path_planner.enable_lf();

    
    while (true)    
    {
      float position = 0.5*line_sensor.right_position;    

      float radius  = estimate_turn_radius(position, 1.0/r_max);
      radius = -sgn(position)*clip(2.0*radius, r_min, r_max);      
      
      path_planner.set_circle_motion(radius, velocity);
      //path_planner.set_circle_motion_trajectory(radius, velocity);
      timer.delay_ms(4);  
    }
    
    return 0;
}
  