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
 

  while (key != 0)
  { 
    led = 1; 
    timer.delay_ms(100);

    led = 0; 
    timer.delay_ms(200);
  }
  
  led = 1; 

  while (key == 0)
  {
    led = 1; 
    timer.delay_ms(50);

    led = 0; 
    timer.delay_ms(50);
  } 

  led = 1; 

  timer.delay_ms(1500);

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