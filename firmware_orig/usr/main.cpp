#include "device.h"
#include <fmath.h>
#include <drivers.h>
#include <tests.h>
#include <identification.h>
#include <uid.h>




int main(void)      
{ 
  drivers_init();
  
  //main LQR controller init
  //position_control.init();  

  //LineFollowing line_following;

  terminal << "\n\n\n"; 
  terminal << "machine ready\n";

  Gpio<TGPIOC, 4, GPIO_MODE_OUT> line_led;   //line ilumination leds
  line_led = 1; 
  

  button();
  

  timer.delay_ms(500); 
 

  motor_control.set_right_velocity(50*2.0*PI/60.0);
  motor_control.set_left_velocity(50*2.0*PI/60.0);
  
  motors_test();  
  //encoder_sensor_test();
  //motor_identification();
  //gyro_stabilisation_test();
  //encoder_sensor_test();
  //gyro_sensor_test(); 
  //ir_sensor_test();
  //line_sensor_test();
   
  while (1)    
  {
    timer.delay_ms(150);

    //terminal << "encoder  = " << motor_control.get_left_position() << " " << motor_control.get_right_position() << "\n";
    terminal << "velocity = ";
    terminal << motor_control.get_left_velocity()*(60.0f/(2.0f*PI)) << " ";
    terminal << motor_control.get_left_velocity_fil()*(60.0f/(2.0f*PI)) << " ";
    terminal << motor_control.get_right_velocity()*(60.0f/(2.0f*PI)) << " ";
    terminal << motor_control.get_right_velocity_fil()*(60.0f/(2.0f*PI)) << " ";
    terminal << "\n";
    
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
  
  

  return 0;
} 