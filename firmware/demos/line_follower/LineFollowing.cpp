#include "LineFollowing.h"

void LineFollowing::init()
{
    this->speed_min = 500.0;     

    this->speed_max = 600.0;
    //this->speed_max = 800.0;
    //this->speed_max = 1000.0;
    //this->speed_max = 1200.0;  
        
    this->r_min   = 80.0;
    this->r_max   = 10000.0;          

    this->qr_max  = 10.0;         
    this->qr_min  = 2.0;  

    led = 0;
    this->steps = 0;

    path_planner.init();
    split_line_detector.init(30.0, 0.9);
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
            split_line_detector.reset();
        }
        
        // lost line search
        while (line_sensor.line_lost_type != LINE_LOST_NONE)   
        {
          float curvature = q_estimator.get_curvature();

          path_planner.disable_lf();
          line_search(line_sensor.line_lost_type, curvature);
          path_planner.enable_lf();

          q_estimator.reset(); 
          split_line_detector.reset();
        }   
        
        /*
        // this detects if there is splited line on and voids robot to cycle on loop forewer
        int split_detection = split_line_detector.step(path_planner.position_control.get_distance(), line_sensor.left_position, line_sensor.right_position);

        if (split_detection != 0)   
        {
          float target_distance = path_planner.position_control.get_distance() + 80.0;

          while (path_planner.position_control.get_distance() < target_distance)
          { 
            path_planner.set_circle_motion(sgn(split_detection)*r_min, speed_min);
            timer.delay_ms(4);      
          }       

          q_estimator.reset();
        } 
        */

        // main line following
        {
          float position = 0.4*line_sensor.right_position;    
          //float position = line_sensor.right_position;    

          float radius  = estimate_turn_radius(position, 1.0/r_max);
          radius = -sgn(position)*clip(radius, r_min*0.1, r_max);    

          float d = path_planner.position_control.get_distance(); 
          q_estimator.add(d, line_sensor.right_position, radius);

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
          float speed = (1.0 - q)*this->speed_min + q*this->speed_max;  

          path_planner.set_circle_motion_trajectory(radius, speed);
          //path_planner.set_circle_motion(radius, speed);
          timer.delay_ms(4); 
        }
    }   

    
    return 0;
}



void LineFollowing::line_search(uint32_t line_lost_type, float curvature)
{
    float turn_search_distance      = 50.0;
    float turn_search_distance_long = 80.0;
    float forward_search_distance   = 70.0;

    float r_search  = 90.0;
    float r_max     = 10000.0;

    float speed     = 500.0;

    int state       = 2;
    int way         = 1; 

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
      if (curvature > 0)
      {
        way = 1;
      }
      else
      {
        way = -1;
      }
      
      state = 2;
    } 

  
    while (true)
    {
        // left or right line searching
        if (state == 0 || state == 1)
        {
            // stop motors
            float d_target  = path_planner.position_control.get_distance();
            float a_target  = path_planner.position_control.get_angle();

            while (path_planner.position_control.get_velocity() > 10.0)
            { 
                path_planner.set_position(d_target, a_target);
                timer.delay_ms(4);      

                led_blink();
            }   

            //turn until line found, or distance trehold
            float start_distance      = path_planner.position_control.get_distance();
            float target_distance     = start_distance + turn_search_distance;

            while (path_planner.position_control.get_distance() < target_distance)
            { 
                path_planner.set_circle_motion(way*r_search, 0.25*speed);
                timer.delay_ms(4);      

                if (line_sensor.line_lost_type == LINE_LOST_NONE)
                {
                  return; 
                } 

                led_blink();
            }       

            while (path_planner.position_control.get_distance() > start_distance)
            { 
                path_planner.set_circle_motion(way*r_search, -0.25*speed);
                timer.delay_ms(4);    
                
                if (line_sensor.line_lost_type == LINE_LOST_NONE)
                {
                  return;
                }

                led_blink();
            }     

            way*= -1;

            state++;
        }
        // line lost in midle, center
        // go forward, until line found or maximal distance reached
        else if (state == 2)  
        {          
            float start_distance      = path_planner.position_control.get_distance();
            float target_distance     = start_distance + forward_search_distance;

            while (path_planner.position_control.get_distance() < target_distance)
            {   
                path_planner.set_circle_motion(r_max, speed);
                timer.delay_ms(4);  

                if (line_sensor.line_lost_type == LINE_LOST_NONE)
                {
                  return; 
                }

                led_blink();
            }       
            
            state = 0;  
        }
        // wave motion to find line
        else
        {
          float search_distance = 0.5*turn_search_distance;
          while (1)
          {
            float start_distance      = path_planner.position_control.get_distance();
            float target_distance     = start_distance + search_distance;

            while (path_planner.position_control.get_distance() < target_distance)
            {   
                path_planner.set_circle_motion(r_search, 0.5*speed_min);
                timer.delay_ms(4);  

                if (line_sensor.line_lost_type == LINE_LOST_NONE)
                {
                  return; 
                }   
 
                led_blink();
            }  

            search_distance = turn_search_distance;
          
            {
              float start_distance      = path_planner.position_control.get_distance();
              float target_distance     = start_distance + search_distance;
            
              while (path_planner.position_control.get_distance() < target_distance)
              {   
                  path_planner.set_circle_motion(-r_search, 0.5*speed_min);
                  timer.delay_ms(4);  

                  if (line_sensor.line_lost_type == LINE_LOST_NONE)
                  {
                    return; 
                  }

                  led_blink();
              }  
            }
          }
        }
    }
}



void LineFollowing::obstacle_avoid()
{
    float r_max   = 10000.0;
    float r_min   = 680.0; 

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

        float r = 1.0/(abs(0.004*diff) + 0.000001); //0.002
        
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



void LineFollowing::curtain_avoid()
{
  float curtain_distance  = 250.0;
  float target_distance   = curtain_distance + path_planner.position_control.get_distance();

  while (path_planner.position_control.get_distance() < target_distance)
  {
    float position = 0.4*line_sensor.right_position;    

    float radius  = estimate_turn_radius(position, 1.0/r_max);
    radius = -sgn(position)*clip(radius, r_min, r_max);   

    float d = path_planner.position_control.get_distance(); 
    q_estimator.add(d, line_sensor.right_position, radius);

    // estimate line straightness
    float q = q_estimator.process();

    //if quality is high (close to 1), increase radius - allows faster speed
    float kr = q*this->qr_max + (1.0 - q)*this->qr_min;  
    radius = kr*radius;
    
    //if quality is high (close to 1), use higher speed
    float speed = q*this->speed_max + (1.0 - q)*this->speed_min;  

    //path_planner.set_circle_motion_trajectory(radius, speed);
    path_planner.set_circle_motion(radius, speed);
    timer.delay_ms(4); 
  }
}



float LineFollowing::estimate_turn_radius(float sensor_reading, float eps)
{
  float x = SENSORS_DISTANCE;
  float y = 0.5*SENSORS_BRACE*abs(sensor_reading);

  float r = (y*y + x*x)/(2.0*y + eps);

  return r;
}


void LineFollowing::led_blink(uint32_t count)
{
  if ((steps%count) < count/4)
  {
    led = 1;
  }
  else
  {
    led = 0;
  }

  this->steps++;
}