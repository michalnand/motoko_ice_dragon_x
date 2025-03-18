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

    /*
    // q = [ 1.0, 1.0, 0.0, 0.0]
    // r = [10.0**5, 10.0]
    float k[] = {
            0.0031256222, 0.0, 0.03428018, 0.0, 
            0.0, 0.3073647, 0.0, 0.57149106 };

    controller.init(k, 1.0);
    */


    // q = [ 1.0, 1.0, 0.0, 0.0]
    // r = [10.0**6, 10.0]
    float k[] = {
		0.0009960027, 0.0, 0.011780507, 0.0, 
		0.0, 0.3073647, 0.0, 0.57149106 };
    
    controller.init(k, 1.0);

    /*
    float k[] = {
		9.801003e-05, 0.0, 0.0011702038, 0.0, 
		0.0, 0.09085617, 0.0, 0.17096679 };

    float ku[] = {
            0.04019666, 0.0, 
            0.0, 0.19127911 };

    
    controller.init(k, ku, 1.0, 1.0);
    */

    this->distance      = 0.0;
    this->angle         = 0.0;
    this->velocity      = 0.0;
    this->angular_rate  = 0.0;

    this->line_angle_prev = 0.0;
    this->line_angle      = 0.0;
    this->lf_mode         = false;

    this->u_forward     = 0.0;
    this->u_turn        = 0.0;

    timer_init();        
}

void PositionControl::set_desired(float distance, float angle)
{
    controller.xr[0] = distance;
    controller.xr[1] = angle;
    controller.xr[2] = 0;
    controller.xr[3] = 0;
}



void PositionControl::callback()
{   
    // load current state 
    float right_position = motor_control.get_right_position_smooth();
    float left_position  = motor_control.get_left_position_smooth();
    
    float right_velocity = motor_control.get_right_velocity_smooth();
    float left_velocity  = motor_control.get_left_velocity_smooth();
    
    //estimate current state 
    this->distance      = 0.25*(right_position + left_position)*WHEEL_DIAMETER;
    this->angle         = 0.5*(right_position - left_position)*WHEEL_DIAMETER / WHEEL_BRACE;

    this->velocity      = 0.25*(right_velocity + left_velocity)*WHEEL_DIAMETER;
    this->angular_rate  = 0.5*(right_velocity - left_velocity)*WHEEL_DIAMETER / WHEEL_BRACE;

    this->line_angle_prev   = this->line_angle;
    this->line_angle        = line_sensor.left_angle;


    // update current state     
    controller.x[0] = this->distance;
    controller.x[1] = this->angle;      
    controller.x[2] = this->velocity*0.000001f*MOTOR_CONTROL_DT;

    if (this->lf_mode)  
    {
        controller.x[3] = this->line_angle - this->line_angle_prev;
    }
    else
    {
        controller.x[3] = this->angular_rate*0.000001f*MOTOR_CONTROL_DT;
    }


    // compute controller output
    controller.step();    

    // send to motors   
    float u_forward  = controller.u[0];
    float u_turn     = controller.u[1];      

    float u_right   = u_forward + u_turn;
    float u_left    = u_forward - u_turn;

    this->u_forward = u_forward;
    this->u_turn    = u_turn;

    float max_rpm = 2000;

    // send to motors
    motor_control.set_left_velocity(u_left*max_rpm*2.0*PI/60.0);
    motor_control.set_right_velocity(u_right*max_rpm*2.0*PI/60.0);

    steps++;
}

float PositionControl::get_distance()
{
    return this->distance;
}

float PositionControl::get_angle()
{
    return this->angle;
}

float PositionControl::get_velocity()
{
    return this->velocity;
}

float PositionControl::get_angular_velocity()
{
    return this->angular_rate;
}


float PositionControl::get_u_forward()
{
    return this->u_forward;
}

float PositionControl::get_u_turn()
{
    return this->u_turn;
}

int  PositionControl::get_forward_saturation()
{
    return controller.saturation[0];
}

int  PositionControl::get_turn_saturation()
{
    return controller.saturation[1];
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

