#include "position_control.h"
#include <drivers.h>

PositionControl *g_position_control_ptr;


#ifdef __cplusplus
extern "C" {
#endif

// timer 7 interrupt handler, running velocity control
void TIM7_IRQHandler(void)
{ 
    //g_position_control_ptr->callback();
    //TIM_ClearITPendingBit(TIM7, TIM_IT_Update);  

    if (TIM7->SR & TIM_SR_UIF)  // Ensure update interrupt flag is set
    {
        TIM7->SR &= ~TIM_SR_UIF; // Clear the interrupt flag
        g_position_control_ptr->callback();
    }
} 

#ifdef __cplusplus
}
#endif  




void PositionControl::init()
{
    g_position_control_ptr = this;

    steps = 0;

    float k[] = {
            0.010153214, -0.008095111, 0.07932922, -0.008937809, 
            0.1072366, 0.40745917, 0.68441015, 0.4456289 };

    float ki[] = {
            0.0002727246, -0.00013918843, 0.0, 0.0, 
            0.004262683, 0.008739533, 0.0, 0.0 };


    lqr.init(k, ki, 1.0);         

    //turn_pid.init(50, 2.0, 4.0,    2000*(2.0*PI)/60.0, 500*(2.0*PI)/60.0);
    //forward_pid.init(10, 2.0, 4.0, 2000*(2.0*PI)/60.0, 1*(2.0*PI)/60.0);   

    this->distance_prev = 0.0;
    this->distance      = 0.0;
    this->angle_prev    = 0.0;
    this->angle         = 0.0;


    timer_init();        
}

void PositionControl::set_single_point(float distance, float angle)
{
    lqr.xr[0] = distance;
    lqr.xr[1] = angle;
    lqr.xr[2] = 0;
    lqr.xr[3] = 0;
}



void PositionControl::callback()
{   
    //estimate current state 
    float right_position = motor_control.get_right_position_smooth();
    float left_position  = motor_control.get_left_position_smooth();

    
    this->distance_prev = this->distance;   
    this->distance      = 0.25*(right_position + left_position)*WHEEL_DIAMETER;

    this->angle_prev    = this->angle;
    this->angle         = 0.5*(right_position - left_position)*WHEEL_DIAMETER / WHEEL_BRACE;

    
    // update current state 
    lqr.x[0] = this->distance;
    lqr.x[1] = this->angle;     
    lqr.x[2] = (this->distance  - this->distance_prev);
    lqr.x[3] = (this->angle     - this->angle_prev);



    // compute controller output
    lqr.step();    
    
    // send to motors
    float u_forward  = lqr.u[0];
    float u_turn     = lqr.u[1];    

    float u_right = u_forward + u_turn;
    float u_left  = u_forward - u_turn;

    float max_rpm = 2000;

    // send to motors
    motor_control.set_left_velocity(u_left*max_rpm*2.0*PI/60.0);
    motor_control.set_right_velocity(u_right*max_rpm*2.0*PI/60.0);

    steps++;
}

float PositionControl::get_distance()
{
    return lqr.x[0];
}

float PositionControl::get_angle()
{
    return lqr.x[1];
}

float PositionControl::get_velocity()
{
    return lqr.x[2];
}

float PositionControl::get_angular_velocity()
{
    return lqr.x[3];
}


void PositionControl::timer_init()
{
    //init timer 7 interrupt for periodic callback
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);    


    TIM_TimeBaseStructure.TIM_Prescaler         = 16 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;   
    TIM_TimeBaseStructure.TIM_Period            = ((POSITION_CONTROL_DT*(APB1Clock/(uint32_t)1000000))/16) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0;        
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    

    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM7, ENABLE);    

    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

