#include "device.h"
#include <fmath.h>
#include <drivers.h>
#include <tests.h>
#include <identification.h>





int main(void)      
{ 
  drivers_init();
  
  //main LQR controller init
  //position_control.init();  

  //LineFollowing line_following;

  terminal << "\n\n\n"; 
  terminal << "machine ready\n";
  

  Gpio<TGPIOB, 2, GPIO_MODE_OUT> led;   //user led
  led = 1;  

  Gpio<TGPIOB, 10, GPIO_MODE_IN_PULLUP> key;   //user button

  timer.delay_ms(200);

  Gpio<TGPIOC, 4, GPIO_MODE_OUT> line_led;   //line iluminatio leds
  line_led = 1; 
  
  
  //wait for buttom press
  while ((int)key != 0)
  { 
    led = 1; 
    timer.delay_ms(100);

    led = 0; 
    timer.delay_ms(200);
  }
  
  led = 1; 

  //wait for button release
  while ((int)key == 0)   
  { 
    led = 1; 
    timer.delay_ms(50);

    led = 0; 
    timer.delay_ms(50);
  } 

  led = 1; 

  timer.delay_ms(500);
  
  
  //motors_test();
  motor_identification();


  

  
  while (1)   
  {
    led = 1; 
    timer.delay_ms(50);
    led = 0; 
    timer.delay_ms(50);

    //terminal << "encoder  = " << motor_control.get_left_position() << " " << motor_control.get_right_position() << "\n";
    //terminal << "velocity = " << motor_control.get_left_velocity() << " " << motor_control.get_right_velocity() << "\n";
  }




  //mcu_usage();
  //noise_measurement();

  //timer_test();

  //left_motor_connect_test();
  //right_motor_connect_test();

  //ir_sensor_test();
  //line_sensor_test();
  //gyro_sensor_test(); 
  //encoder_sensor_test();
  //sensors_test();
  

  //left_motor_pwm_test();
  //right_motor_pwm_test();
  //encoder_sensor_test();

  //motor_identification();
  //motor_driver_test();   
  //shaper_test();
  
  //robot_dynamics_identification();


  //line_following.main(); 
  
  //line_following.obstacle_avoid();
  //line_following.line_search(LINE_LOST_CENTER);
  
  

  while (1)   
  {
    led = 1; 
    timer.delay_ms(100);

    led = 0; 
    timer.delay_ms(100);

    led = 1; 
    timer.delay_ms(200);

    led = 0; 
    timer.delay_ms(400);
  }
 
  return 0;
} 