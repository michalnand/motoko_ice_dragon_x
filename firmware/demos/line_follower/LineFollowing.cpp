#include "LineFollowing.h"

void LineFollowing::init()
{
    this->speed_min = 400.0;     
    this->speed_max = 1200.0;         

    this->r_min = 80.0;
    this->r_max = 10000.0;        


    this->q_penalty = 3.0;      
    this->qr_max    = 8.0;      
    this->qr_min    = 1.5;  

    path_planner.init();

}

int LineFollowing::main()
{
    quality_filter.init(1.0);


    path_planner.enable_lf();

    while (true)
    {
        /*
        int obstacle = ir_sensor.obstacle_detected();

        //obstacle avoiding
        if (obstacle == 2) 
        {
          if (obstacle_map[this->obstacle_idx] == true)
          {
            position_control.disable_lf();
            obstacle_avoid(); 
            position_control.enable_lf(); 

            speed_min_curr = 0.0; 
            quality_filter.init(1.0);
          }
          else
          {
            curtain_avoid();

            speed_min_curr = speed_min; 
            quality_filter.init(1.0);
          } 

          split_line_detector.reset();  

          this->obstacle_idx = (this->obstacle_idx+1)%obstacle_map.size();
        }
        
        //lost line search
        while (line_sensor.line_lost_type != LINE_LOST_NONE)   
        {
          position_control.disable_lf();
          line_search(line_sensor.line_lost_type);
          position_control.enable_lf();

          quality_filter.init(1.0); 
          split_line_detector.reset();
        }   
        */


        //main line following
        float position = 0.5*line_sensor.right_position;    

        float radius  = estimate_turn_radius(position, 1.0/r_max);
        radius = -sgn(position)*clip(radius, r_min, r_max);   
        
        // estimate line straightness
        quality_filter.step(abs(position));

        float q = 1.0 - this->q_penalty*quality_filter.max();   
        q = clip(q, 0.0, 1.0);    
        
        //if quality is high (close to 1), increase radius - allows faster speed
        float kr = q*this->qr_max + (1.0 - q)*this->qr_min;  

        radius = kr*radius;

        //if quality is high (close to 1), use higher speed
        float speed = q*this->speed_max + (1.0 - q)*this->speed_min;  

        path_planner.set_circle_motion_trajectory(radius, speed);
        timer.delay_ms(4); 
    }   

    
    return 0;
}






/*
void LineFollowing::line_search(uint32_t line_lost_type)
{
  float search_distance = 120.0;

  uint32_t state = 0; 
  int      way   = 1;

  if (line_lost_type == LINE_LOST_LEFT)
  {
    way   = 1;
    state = 0;
  }
  else if (line_lost_type == LINE_LOST_RIGHT)
  {
    way   = -1;
    state = 0; 
  }
  else
  { 
    way = 1;
    state = 2;
  }
  
  while (1)
  {
    if (state == 0 || state == 1)
    {
      //turn until line found, or dostance trehold
      float start_distance  = position_control.distance;
      float target_distance = start_distance + search_distance;

      while (position_control.distance < target_distance)
      { 
        position_control.set_circle_motion(way*2.0*r_min, speed_min);
        timer.delay_ms(4);    

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      }  

      while (position_control.distance > start_distance)
      { 
        position_control.set_circle_motion(-way*2.0*r_min, speed_min);
        timer.delay_ms(4);    

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      } 

      way*= -1;
      state++;
    }

    //go forward, until line found or maximal distance reached
    else
    {
      float target_distance = position_control.distance + search_distance*0.7;
      while (position_control.distance < target_distance)
      { 
        position_control.set_circle_motion(r_max, speed_min);
        timer.delay_ms(4);  


        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      }   

      float start_angle = position_control.angle;
      float target_angle;

      target_angle = start_angle - 0.5*PI;

      while (abs(position_control.angle - target_angle) > 0.02*PI)
      { 
        position_control.set(position_control.distance, target_angle);
        timer.delay_ms(4);      

        
        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        } 
      }   

      target_angle = start_angle + 0.5*PI;

      while (abs(position_control.angle - target_angle) > 0.02*PI)
      { 
        position_control.set(position_control.distance, target_angle);
        timer.delay_ms(4);  

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        } 
      } 

      target_angle = start_angle;

      while (abs(position_control.angle + target_angle) > 0.1*PI)
      { 
        position_control.set(position_control.distance, target_angle);
        timer.delay_ms(4);  

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      } 

      state = 0;  
    }
  }
}
*/


/*
void LineFollowing::obstacle_avoid()
{
  float r_min = 550; //550.0;  

  if (FAST_RUN)
  {
    r_min = 700.0;
  } 
  else  
  {
    r_min = 750.0;  
  }

  
  float r_max = 10000.0;
  float speed = speed_min;  
  float d_req = 80.0;      
  float r_turn= 160.0;    

  float target_distance = position_control.distance - 10.0;  
  float speed_curr = 0.0;   

  while (ir_sensor.obstacle_distance() < 50.0 || position_control.distance > target_distance) 
  {
    speed_curr = clip(speed_curr + 1.0, 0.0, speed);
    position_control.set(position_control.distance - speed_curr, position_control.angle);
    timer.delay_ms(4);       
  } 

  //turn left, 90degrees 
  float angle_target = position_control.angle + PI/2.0;

  while (angle_target > position_control.angle)
  {
    position_control.set_circle_motion(r_turn, speed);
    timer.delay_ms(4);   
  } 

  //turn around bostacle, circular motion
  uint32_t state = 0;  
  float angle_mark = position_control.angle - 0.6*PI;
  
  while (1)   
  {
    if (state == 0 && position_control.angle < angle_mark)
    {
      state = 1;    
    }
    else if (state == 1 && line_sensor.line_lost_type == LINE_LOST_NONE)
    {
      break; 
    }   

    float diff   = d_req - ir_sensor.get()[3];  

    diff = clip(diff, -150.0, 150.0);               

    float r = 1.0/(abs(0.00005*diff) + 0.00000001);     

    r = sgn(diff)*clip(r, r_min, r_max);

    position_control.set_circle_motion(r, speed);
    timer.delay_ms(4);   
  }

  //turn left, 90degrees  
  angle_target = position_control.angle + PI/2.0;
  while (angle_target > position_control.angle)
  {
    position_control.set_circle_motion(r_turn, speed);
    timer.delay_ms(4);   
  }   
}
*/

float LineFollowing::estimate_turn_radius(float sensor_reading, float eps)
{
  float x = SENSORS_DISTANCE;
  float y = 0.5*SENSORS_BRACE*abs(sensor_reading);

  float r = (y*y + x*x)/(2.0*y + eps);

  return r;
}