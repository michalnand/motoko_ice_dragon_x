#include "device.h"
#include <fmath.h>
#include <drivers.h>
#include <tests.h>
#include <identification.h>


#ifdef __cplusplus
extern "C" {
#endif

volatile int32_t g_left_encoder, g_right_encoder;


// encoder interrupt handling
void EXTI15_10_IRQHandler(void)
{
  // right encoder
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11))
    {
      g_left_encoder++;  
    }
    else
    {
      g_left_encoder--;
    }   

    EXTI_ClearITPendingBit(EXTI_Line10);
  }


  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
    {
      g_right_encoder++;  
    }
    else
    {
      g_right_encoder--;
    }   

    EXTI_ClearITPendingBit(EXTI_Line12);
  }  
}


#ifdef __cplusplus
}
#endif


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

  timer.delay_ms(10);
  
  /*
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
  */
  
  //motors_test();

  //encoder init
  Gpio<TGPIOC, 10, GPIO_MODE_IN_PULLUP> enc_left_a; 
  Gpio<TGPIOC, 11, GPIO_MODE_IN_PULLUP> enc_left_b; 

  Gpio<TGPIOC, 12, GPIO_MODE_IN_PULLUP> enc_right_a; 
  Gpio<TGPIOB, 5, GPIO_MODE_IN_PULLUP> enc_right_b; 

  EXTI_InitTypeDef   EXTI_InitStructure;
  NVIC_InitTypeDef   NVIC_InitStructure;


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  // EXTI15_10_IRQn Line to PC10 pin 
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource10);

  // Configure EXTI15_10_IRQn line  
  EXTI_InitStructure.EXTI_Line    = EXTI_Line10;  
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;     
  EXTI_Init(&EXTI_InitStructure);   


  // EXTI15_10_IRQn Line to PC12 pin 
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource12);

  // Configure EXTI15_10_IRQn line  
  EXTI_InitStructure.EXTI_Line    = EXTI_Line12;  
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;     
  EXTI_Init(&EXTI_InitStructure);        

  // Enable and set EXTI15_10_IRQn Interrupt           
  NVIC_InitStructure.NVIC_IRQChannel          = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd       = ENABLE;
  NVIC_Init(&NVIC_InitStructure);



  g_left_encoder = 0;
  g_right_encoder = 0;
  
  while (1)   
  {
    led = 1; 
    timer.delay_ms(50);
    led = 0; 
    timer.delay_ms(50);

    //terminal << "encoder = " << g_left_encoder << " " << (int)enc_left_a << " " << (int)enc_left_b << "\n";
    terminal << "encoder = " << g_left_encoder << " " << g_right_encoder << "\n";
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