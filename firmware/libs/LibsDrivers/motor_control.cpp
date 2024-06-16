#include <drivers.h>
#include <motor_control.h>

 

MotorControl *g_motor_control_ptr;

#ifdef __cplusplus
extern "C" {
#endif

//encoder interrupt handling
void EXTI0_IRQHandler(void)
{
    g_motor_control_ptr->left_enc++;
    g_motor_control_ptr->right_enc++;
}

//timer 2 interrupt handler, running velocity control
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
    this->left_ol_mode = true;
    this->right_ol_mode = true;

    this->left_torque         = 0;
    this->right_torque        = 0;
    this->left_req_velocity   = 0;
    this->right_req_velocity  = 0;

    this->left_position  = 0;
    this->left_velocity  = 0;
    this->right_position = 0;
    this->right_velocity = 0;

    this->left_enc  = 0; 
    this->right_enc = 0;



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

// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void  MotorControl::set_left_torque(float left_torque)
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



//wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_left_position()
{
    return this->left_position;
}

//wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
float MotorControl::get_left_velocity()
{
    return this->left_velocity;
}

//wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_right_position()
{
    return this->right_position;
}

//wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
float MotorControl::get_right_velocity()
{
    return this->right_velocity;
}



void MotorControl::callback()
{
    float left_velocity  = -left_encoder.angular_velocity*2.0*PI/ENCODER_RESOLUTION;
    float right_velocity = right_encoder.angular_velocity*2.0*PI/ENCODER_RESOLUTION;

    this->left_torque  = MOTOR_CONTROL_MAX*left_controller.step(left_req_velocity, left_velocity);
    this->right_torque = MOTOR_CONTROL_MAX*right_controller.step(right_req_velocity, right_velocity);
 
    this->left_torque  = clamp(this->left_torque,  -MOTOR_CONTROL_MAX, MOTOR_CONTROL_MAX);
    this->right_torque = clamp(this->right_torque, -MOTOR_CONTROL_MAX, MOTOR_CONTROL_MAX);

   
    set_torque_from_rotation(this->left_torque,    false,  left_encoder.angle,   0);
    set_torque_from_rotation(-this->right_torque,  false, right_encoder.angle,  1);
}