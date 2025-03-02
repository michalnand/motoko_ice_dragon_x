#include "position_control.h"
#include <drivers.h>

PositionControl *g_position_control_ptr;


#ifdef __cplusplus
extern "C" {
#endif

// timer 7 interrupt handler, running velocity control
void TIM7_IRQHandler(void)
{ 
    g_position_control_ptr->callback();
    //TIM_ClearITPendingBit(TIM7, TIM_IT_CC1);  
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);  
} 

#ifdef __cplusplus
}
#endif




void PositionControl::init()
{
    g_position_control_ptr = this;

    velocity_max = 4000*PI/180.0;   

    distance_pid.init(200.0, 0*0.00025, 200.0,   velocity_max, 0.01);
    angle_pid.init(60.0, 0.03, 100.0,  velocity_max, 10000.0);


    timer_init();   
}

void PositionControl::set_single_point(float distance, float angle)
{
    this->distance = distance;
    this->angle    = angle;
}


float PositionControl::get_distance()
{
    return 0.5*(motor_control.get_left_position() + motor_control.get_right_position())*0.5*WHEEL_DIAMETER;
}

float PositionControl::get_velocity()
{
    return 0.5*(motor_control.get_left_velocity() + motor_control.get_right_velocity())*0.5*WHEEL_DIAMETER;
}

float PositionControl::get_angle()  
{
    return (motor_control.get_left_position() - motor_control.get_right_position())*0.5*WHEEL_DIAMETER/WHEEL_BRACE;
}

float PositionControl::get_angular_velocity()
{
    return (motor_control.get_left_velocity() - motor_control.get_right_velocity())*0.5*WHEEL_DIAMETER/WHEEL_BRACE;
}

void PositionControl::callback()
{
    float u_angle    = angle_pid.step(angle, this->get_angle());
    float u_forward  = distance_pid.step(distance, this->get_distance());

    
    // limit angular velocity
    u_angle = clip(u_angle, -velocity_max, velocity_max);

    // limit forward velocity
    float max_forward = velocity_max - abs(u_angle);

    u_forward = sgn(u_forward) * min(abs(u_forward), max_forward);
    


    float u_left  = u_forward + u_angle;
    float u_right = u_forward - u_angle;
    
    // send to motors
    motor_control.set_left_velocity(u_left);
    motor_control.set_right_velocity(u_right);
}



void PositionControl::timer_init()
{
    //init timer 7 interrupt for periodic callback
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);    


    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = timer_period(POSITION_CONTROL_DT);
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    //TIM_ITConfig(TIM7, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);   

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    

}

