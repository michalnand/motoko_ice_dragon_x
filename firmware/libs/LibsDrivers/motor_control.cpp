#include <drivers.h>
#include <motor_control.h>

 

MotorControl *g_motor_control_ptr;

#ifdef __cplusplus
extern "C" {
#endif

volatile int32_t  g_left_encoder, g_right_encoder;
volatile uint64_t g_left_time_now, g_left_time_prev;
volatile uint64_t g_right_time_now, g_right_time_prev;
volatile int32_t  g_left_velocity, g_right_velocity;

// encoder interrupt handling
void EXTI15_10_IRQHandler(void)
{
  // left encoder
  if(EXTI_GetITStatus(EXTI_Line10) != RESET)
  {
    g_left_time_prev = g_left_time_now;
    g_left_time_now  = timer.get_ns_time();
    g_left_velocity  = 1000000000/(g_left_time_now - g_left_time_prev);

    

    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11))
    {
      g_left_encoder++;  
    }
    else
    {
      g_left_encoder--;
      g_left_velocity = -g_left_velocity;
    }   

    

    EXTI_ClearITPendingBit(EXTI_Line10);
  }

  // right encoder
  if(EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
    g_right_time_prev = g_right_time_now;
    g_right_time_now  = timer.get_ns_time();
    g_right_velocity  = 1000000000/(g_right_time_now - g_right_time_prev);

    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5))
    {
      g_right_encoder++;  
    }
    else
    {
      g_right_encoder--;
      g_right_velocity = -g_right_velocity;
    }   

    EXTI_ClearITPendingBit(EXTI_Line12);
  }  
}



// timer 2 interrupt handler, running velocity control
void TIM2_IRQHandler(void)
{ 
    g_motor_control_ptr->callback();
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);  
} 


#ifdef __cplusplus
}
#endif


//init motor control process
void MotorControl::init()
{
    g_motor_control_ptr = this;


    this->left_ol_mode  = true;
    this->right_ol_mode = true;

    this->left_torque         = 0;
    this->right_torque        = 0;
    this->left_req_velocity   = 0;
    this->right_req_velocity  = 0;

    left_pwm.init();
    right_pwm.init();

    //optimal control init 
    
    //discrete dynamics model
    float a = 0.95484104;
    float b = 7.08995665;

    //LQR gain, q = 1.0, r = 1*10**7
    float k  =  0.00514003;
    float ki =  0.00032194;

    //Kalman gain  
    float f  =  0.00818888;

    left_controller.init(a, b, k, ki, f, MOTOR_CONTROL_MAX_VELOCITY);
    right_controller.init(a, b, k, ki, f, MOTOR_CONTROL_MAX_VELOCITY);

    //init encoders
    encoder_init();

    //init timer
    timer_init();
}


// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void MotorControl::set_left_torque(float left_torque)
{
    this->left_ol_mode = true;
    this->left_torque  = left_torque;
}

// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void  MotorControl::set_right_torque(float right_torque)
{
    this->right_ol_mode = true;
    this->right_torque  = right_torque;
}


// turn ON closed loop control, and set required velocity in rad/s
// max velocity is arround 1000RPM (2PI*1000/60 [rad/s])
void MotorControl::set_left_velocity(float left_velocity)
{
    this->left_req_velocity  = left_velocity;
    this->left_ol_mode       = false;
}

// turn ON closed loop control, and set required velocity in rad/s
// max velocity is arround 1000RPM (2PI*1000/60 [rad/s])
void MotorControl::set_right_velocity(float right_velocity)
{
    this->right_req_velocity  = right_velocity;
    this->right_ol_mode       = false;
}

// force break to both motors
void MotorControl::halt()
{
    this->set_left_velocity(0);
    this->set_right_velocity(0);
}

// wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_left_position()
{
    return (2.0f*PI*g_left_encoder)/ENCODER_RESOLUTION;
}

// wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
float MotorControl::get_left_velocity()
{
    return (4.0f*PI*g_left_velocity)/ENCODER_RESOLUTION;
}

// wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_right_position()
{
    return (2.0f*PI*g_right_encoder)/ENCODER_RESOLUTION;
}

// wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
float MotorControl::get_right_velocity()
{
    return (4.0f*PI*g_right_velocity)/ENCODER_RESOLUTION;
}        
 



void MotorControl::callback()
{
    float left_torque = 0;
    float right_torque = 0;

   

    if (this->left_ol_mode)
    {
        left_torque = this->left_torque;
    }
    else
    {
        //TODO add velocity controller step here
        left_torque = 0;
    }

    if (this->right_ol_mode)
    {
        right_torque = this->right_torque;
    }
    else
    {
        //TODO add velocity controller step here
        right_torque = 0;
    }   


    if (left_torque == 0)
    {
        g_left_velocity = (15*g_left_velocity)/16;
    }

    if (right_torque == 0)
    {
        g_right_velocity = (15*g_right_velocity)/16;
    }   

    left_pwm.set(left_torque*(int32_t)PWM_PERIOD);
    right_pwm.set(right_torque*(int32_t)PWM_PERIOD);
}







void MotorControl::encoder_init()
{
    //init encoder variables  
    g_left_encoder    = 0;
    g_right_encoder   = 0;
    g_left_time_now   = 0;
    g_left_time_prev  = 0;
    g_right_time_now  = 0;
    g_right_time_prev = 0;
    g_left_velocity   = 0;
    g_right_velocity  = 0;


    //encoder init
    Gpio<TGPIOC, 10, GPIO_MODE_IN_PULLUP> enc_left_a; 
    Gpio<TGPIOC, 11, GPIO_MODE_IN_PULLUP> enc_left_b; 

    Gpio<TGPIOC, 12, GPIO_MODE_IN_PULLUP> enc_right_a; 
    Gpio<TGPIOB, 5, GPIO_MODE_IN_PULLUP>  enc_right_b; 

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
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;     
    EXTI_Init(&EXTI_InitStructure);        

    // Enable and set EXTI15_10_IRQn Interrupt           
    NVIC_InitStructure.NVIC_IRQChannel          = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd       = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



void MotorControl::timer_init()
{
    //init timer 2 interrupt for periodic callback calling, 4kHz
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = timer_period(MOTOR_CONTROL_DT);
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);   

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

