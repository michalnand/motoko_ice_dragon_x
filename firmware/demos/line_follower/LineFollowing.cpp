#include "LineFollowing.h"

void LineFollowing::init()
{
    this->speed_min = 500.0;     

    this->speed_max = 800.0;
    //this->speed_max = 1000.0;
    //this->speed_max = 1200.0;         

    this->r_min = 80.0;
    this->r_max = 10000.0;          


    this->qr_max    = 10.0;         
    this->qr_min    = 1.5;  

    path_planner.init();
} 

int LineFollowing::main()
{ 
    /*
    while (true)
    {
      if (ir_sensor.obstacle_detected())
      {
        obstacle_avoid(); 
      }
    }
    */
    
    q_estimator.init(1.0, 0.0, 0.0);
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

            q_estimator.reset();
          }
          else
          {
            curtain_avoid();

            q_estimator.reset();
          } 

          //split_line_detector.reset();  

          this->obstacle_idx = (this->obstacle_idx+1)%obstacle_map.size();
        }
        */

        // obstacle avoiding
        int obstacle = ir_sensor.obstacle_detected();

        if (obstacle == 2)
        {
            path_planner.disable_lf();
            obstacle_avoid(); 
            path_planner.enable_lf(); 

            q_estimator.reset();
        }
        
        // lost line search
        while (line_sensor.line_lost_type != LINE_LOST_NONE)   
        {
          path_planner.disable_lf();
          line_search(line_sensor.line_lost_type);
          path_planner.enable_lf();

          q_estimator.reset(); 
          //split_line_detector.reset();
        }     

        // main line following
        {
          float position = 0.4*line_sensor.right_position;    

          float radius  = estimate_turn_radius(position, 1.0/r_max);
          radius = -sgn(position)*clip(radius, r_min, r_max);   

          float d = path_planner.position_control.get_distance(); 
          q_estimator.add(d, line_sensor.right_position);

          // estimate line straightness
          float q = q_estimator.process();

          // obstacle warning, slow down
          if (obstacle != 0)
          {
            q = 0.0;  
          }


          //if quality is high (close to 1), increase radius - allows faster speed
          float kr = q*this->qr_max + (1.0 - q)*this->qr_min;  
          radius = kr*radius;
          
          //if quality is high (close to 1), use higher speed
          float speed = q*this->speed_max + (1.0 - q)*this->speed_min;  

          path_planner.set_circle_motion_trajectory(radius, speed);
          timer.delay_ms(4); 
        }
    }   

    
    return 0;
}



void LineFollowing::line_search(uint32_t line_lost_type)
{
  float turn_search_distance    = 70.0;
  float forward_search_distance = 80.0;

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
    // left or right line searching
    if (state == 0 || state == 1)
    {
      //turn until line found, or distance trehold
      float start_distance  = path_planner.position_control.get_distance();
      float target_distance = start_distance + turn_search_distance;

      while (path_planner.position_control.get_distance() < target_distance)
      { 
        path_planner.set_circle_motion(way*r_min, 0.25*speed_min);
        timer.delay_ms(4);      

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        } 
      }      

      while (path_planner.position_control.get_distance() > start_distance)
      { 
        path_planner.set_circle_motion(way*r_min, -0.25*speed_min);
        timer.delay_ms(4);    

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      }     

      way*= -1;
      state++;
    } 

    // line lost in midle, center
    // go forward, until line found or maximal distance reached
    else
    {
      float target_distance = path_planner.position_control.get_distance() + forward_search_distance;
      float start_angle     = path_planner.position_control.get_angle();
       
      while (path_planner.position_control.get_distance() < target_distance)
      { 
        //path_planner.set_circle_motion(r_max, speed_min);
        path_planner.set_position(target_distance*1.5, start_angle);
        timer.delay_ms(4);  

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      }     
      
     
      // after traveling max distance, try to look at left or right
      float start_distance = path_planner.position_control.get_distance();
      
      start_angle    = path_planner.position_control.get_angle();
      
      float target_angle;

      target_angle = start_angle - 90.0*PI/180.0;

      while (abs(path_planner.position_control.get_angle() - target_angle) > 0.02*PI)
      { 
        path_planner.set_position(start_distance, target_angle);
        timer.delay_ms(4);      

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      }   

      target_angle = start_angle + 90.0*PI/180.0;

      while (abs(path_planner.position_control.get_angle() - target_angle) > 0.02*PI)
      { 
        path_planner.set_position(start_distance, target_angle);
        timer.delay_ms(4);  

        if (line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          return;
        }
      } 
      

      // align robot back
      target_angle = start_angle;

      while (abs(path_planner.position_control.get_angle() + target_angle) > 0.1*PI)
      { 
        path_planner.set_position(start_distance, target_angle);
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



void LineFollowing::obstacle_avoid()
{
    float r_max   = 10000.0;
    float r_min   = 600.0; 

    float speed = speed_min;  
    float d_req = 80.0;      

    // move back until minimal distance from obstalce reached 
    {
      float target_distance = path_planner.position_control.get_distance() - 10.0;  
      
      while (ir_sensor.obstacle_distance() < 50.0 || path_planner.position_control.get_distance() > target_distance) 
      {
        path_planner.set_circle_motion(r_max, -speed_min);
        timer.delay_ms(4);       
      } 
    }

    //turn left, 90degrees 
    {
      float distance_target = path_planner.position_control.get_distance();
      float angle_target    = path_planner.position_control.get_angle() + 90.0*PI/180.0;

      while (abs(path_planner.position_control.get_angle() - angle_target) > 0.02*PI)
      { 
        path_planner.set_position(distance_target, angle_target);
        timer.delay_ms(4);   
      } 
    }
    

    //turn around obstacle, circular motion
    {

      uint32_t state = 0;   
      float angle_start = path_planner.position_control.get_angle();
      float angle_turn  = -90.0*PI/180.0;
      
      while (1)      
      {
        if (state == 0 && (path_planner.position_control.get_angle() - angle_start) < angle_turn)
        {
          state = 1;    
        }
        else if (state == 1 && line_sensor.line_lost_type == LINE_LOST_NONE)
        {
          break; 
        }   

        float diff = d_req - ir_sensor.get()[1];     
        
        diff = clip(diff, -150.0, 150.0);                

        float r = 1.0/(abs(0.001*diff) + 0.000001);     
        
        r = sgn(diff)*clip(r, r_min, r_max);
        
        path_planner.set_circle_motion(r, speed);
        timer.delay_ms(4);    
      } 
    }

    //turn left, 90degrees 
    {
      float distance_target = path_planner.position_control.get_distance();
      float angle_target    = path_planner.position_control.get_angle() + 90.0*PI/180.0;

      while (angle_target > path_planner.position_control.get_angle())
      {
        path_planner.set_position(distance_target, angle_target);
        timer.delay_ms(4);   
      } 
    }
}


float LineFollowing::estimate_turn_radius(float sensor_reading, float eps)
{
  float x = SENSORS_DISTANCE;
  float y = 0.5*SENSORS_BRACE*abs(sensor_reading);

  float r = (y*y + x*x)/(2.0*y + eps);

  return r;
}


void LineFollowing::obstacle_test()
{
  float speed  = speed_min;
  

  while (1)
  {
    int obstacle = ir_sensor.obstacle_detected();
    if (obstacle == 2)
    {
      break;
    }
    
    timer.delay_ms(4);
  }

  obstacle_avoid();
}
  