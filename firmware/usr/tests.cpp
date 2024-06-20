#include <tests.h>
#include <drivers.h>
#include <fmath.h>
#include <shaper.h>
#include <pid.h>


#define LED_GPIO        TGPIOB
#define LED_PIN         2
   


void motors_test()
{
  terminal << "\n\n\nleft motor test\n";
  
  PWMLeft     left_pwm;
  left_pwm.init();

  terminal << "left motor FORWARD speed up\n";
  for (int32_t i = 0; i < (int32_t)PWM_PERIOD; i+= 10)
  {
    left_pwm.set(i);
    timer.delay_ms(5);  
  }

  timer.delay_ms(1000);
  
  terminal << "left motor STOP\n";
  left_pwm.set(0);
  timer.delay_ms(1000);  

  terminal << "left motor BACKWARD speed up\n";
  for (int32_t i = 0; i < (int32_t)PWM_PERIOD; i+= 10)
  {
    left_pwm.set(-i);
    timer.delay_ms(5);  
  }

  timer.delay_ms(1000);
  
  terminal << "left motor STOP\n";
  left_pwm.set(0);
  timer.delay_ms(1000);  
  


  terminal << "\n\n\nright motor test\n";

  PWMRight     right_pwm;

  right_pwm.init();

  terminal << "right motor FORWARD speed up\n";
  for (int32_t i = 0; i < (int32_t)PWM_PERIOD; i+= 10)
  {
    right_pwm.set(i);
    timer.delay_ms(5);  
  }

  timer.delay_ms(1000);
  
  terminal << "right motor STOP\n";
  right_pwm.set(0);
  timer.delay_ms(1000);  

  terminal << "right motor BACKWARD speed up\n";
  for (int32_t i = 0; i < (int32_t)PWM_PERIOD; i+= 10)
  {
    right_pwm.set(-i);
    timer.delay_ms(5);   
  }

  timer.delay_ms(1000);
  
  terminal << "right motor STOP\n";
  right_pwm.set(0);
  timer.delay_ms(1000); 
}


void gyro_sensor_test()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led; 
  led = 1;  

  uint32_t steps = 0;

  float angle = 0.0;
  while (1)
  {
    //read angular rate, rad/s
    float angular_rate = gyro_sensor.read();
    //integrate angular rate into angle, radians
    angle = angle + angular_rate*4.0/1000.0;
    timer.delay_ms(4);

    if ((steps%100) == 0)
    {
      led = 1; 
      terminal << angular_rate << " " << angle*180.0f/PI << "\n";
      led = 0; 
    }
    steps++;
  }
}



void gyro_stabilisation_test()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led; 
  led = 1;  

  uint32_t steps = 0;
  float angle = 0.0;

  PID pid;
  pid.init(0.5, 0.001, 4.0, 1.0);

  while (1)
  {
    float angular_rate = gyro_sensor.read();
    angle = angle + angular_rate*4.0/1000.0;
    timer.delay_ms(4);

    float u = pid.step(0.0, angle);

    motor_control.set_left_torque(-u);
    motor_control.set_right_torque(u);


    if ((steps%100) == 0)
    {
      led = 1; 
      terminal << angular_rate << " " << angle*180.0f/PI << "\n";
      led = 0; 
    }
    steps++;
  }
}


void ir_sensor_test()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
  
  led = 1; 

  while(1)
  {
      uint32_t measurement_id_prev = ir_sensor.measurement_id;
      timer.delay_ms(100);
      uint32_t measurement_id_now = ir_sensor.measurement_id;

      terminal << "measurements/s : " << 10*(measurement_id_now - measurement_id_prev) << "\n";
      terminal << "ir readings    : ";


      for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
      {
        terminal << ir_sensor.get()[i] << " ";
      }
      terminal << ir_sensor.obstacle_detected() << " ";
      terminal << "\n\n\n";
  }
}
 


void line_sensor_test()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led
    
    while(1)
    {
      led = 1; 
      timer.delay_ms(50);

      led = 0; 
      timer.delay_ms(50);
 
      uint32_t measurement_id_prev = line_sensor.measurement_id;
      timer.delay_ms(500);
      uint32_t measurement_id_now  = line_sensor.measurement_id;

      terminal << "measurements/s : " << 2*(measurement_id_now - measurement_id_prev) << "\n";
      line_sensor.print();  

      terminal << "\n\n\n";
  }
}





void encoder_sensor_test()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

    
    while(1)  
    {
      led = 1; 
      timer.delay_ms(50);

      led = 0; 
      timer.delay_ms(450);

      
      terminal << "encoder\n";
      terminal << "left   : " << motor_control.get_left_position()*(float)(180.0/PI)   << " " << motor_control.get_left_velocity()*(float)(60.0/(2.0*PI)) << "\n";
      terminal << "right  : " << motor_control.get_right_position()*(float)(180.0/PI)  << " " << motor_control.get_right_velocity()*(float)(60.0/(2.0*PI)) << "\n";
      terminal << "\n\n\n";

  }
}




void motor_driver_test()
{
    Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;        //user led

    //required RPM velocity
    const float required[] = {0, 10, 50, 200, 1000, 0, -10, -50, -200, -1000};
    //const float required[] = {0, 1500}; 

    while (1)   
    {
      uint32_t time = timer.get_time();
      uint32_t required_idx = (time/4000)%10;

      //convert rpm to rad/s
      float req = required[required_idx]*2.0*PI/60.0;
        
      motor_control.set_left_velocity(req);
      motor_control.set_right_velocity(req);
 

      if ((time/50)%10 == 0)
      { 
        led = 1;  

        terminal << "req   = " <<  req*(float)(60.0/(2.0*PI)) << "\n";  
        terminal << "left  = " <<  motor_control.get_left_velocity()*(float)(60.0/(2.0*PI))  << "\n";
        terminal << "right = " <<  motor_control.get_right_velocity()*(float)(60.0/(2.0*PI)) << "\n";
        terminal << "\n\n"; 
        
      }
      else
      {
        led = 0;
      }
    }
}





void mcu_usage()
{
  Gpio<LED_GPIO, LED_PIN, GPIO_MODE_OUT> led;

  uint32_t counter_no_load = 14391346;

  while(1)
  {
    uint32_t time_start = timer.get_time();
    uint32_t time_stop  = time_start + 1000;
    uint32_t counter = 0;

    led = 1;
    while (timer.get_time() < time_stop)
    {
      counter++;
    }
    led = 0;

    uint32_t cpu_usage = 100 - (100*counter)/counter_no_load;

    terminal << "cpu_usage = " << cpu_usage << " [%]\n";
  }
}




/*
void noise_measurement()
{
  uint32_t n_max = 1000;

  float mean_distance          = 0.0;
  float mean_angle             = 0.0;
  float mean_velocity          = 0.0;
  float mean_angular_velocity  = 0.0;


  float var_distance          = 0.0;
  float var_angle             = 0.0;
  float var_velocity          = 0.0;
  float var_angular_velocity  = 0.0;

  terminal << "measuring mean and variance\n";


  float k = 0.99; 

  for (unsigned int n = 0; n < n_max; n++)
  {
    float d  = position_control.distance; 
    float a  = position_control.angle;
    float dd = position_control.distance - position_control.distance_prev;
    float da = position_control.angle    - position_control.angle_prev;

    mean_distance         = k*mean_distance +         (1.0 - k)*d;
    mean_angle            = k*mean_angle +            (1.0 - k)*a;
    mean_velocity         = k*mean_velocity +         (1.0 - k)*dd;
    mean_angular_velocity = k*mean_angular_velocity + (1.0 - k)*da;


    var_distance         = k*var_distance +         (1.0 - k)*(d - mean_distance)*(d - mean_distance);
    var_angle            = k*var_angle +            (1.0 - k)*(a - mean_angle)*(a - mean_angle);
    var_velocity         = k*var_velocity +         (1.0 - k)*(dd - mean_velocity)*(dd - mean_velocity);
    var_angular_velocity = k*var_angular_velocity + (1.0 - k)*(da - mean_angular_velocity)*(da - mean_angular_velocity);

    timer.delay_ms(4);
  }

  var_distance*= 1000;
  var_angle*= 1000;
  var_velocity*= 1000;
  var_angular_velocity*= 1000;


  terminal << "noise mean\n";
  terminal << mean_distance << " " << mean_angle << " " << mean_velocity << " " << mean_angular_velocity << "\n";
  terminal << "\n\n";

  terminal << "noise var\n";
  terminal << var_distance << " " << var_angle << " " << var_velocity << " " << var_angular_velocity << "\n";
  terminal << "\n\n";

  var_distance         = fsqrt(var_distance);
  var_angle            = fsqrt(var_angle);
  var_velocity         = fsqrt(var_velocity);
  var_angular_velocity = fsqrt(var_angular_velocity);

  terminal << "noise std\n";
  terminal << var_distance << " " << var_angle << " " << var_velocity << " " << var_angular_velocity << "\n";
  terminal << "\n\n";
}
*/