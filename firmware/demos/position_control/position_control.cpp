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
    float k[] = {
        0.005961399, 0.0, 0.059124652, 0.0, 
        0.0, 1.0792116, 0.0, 7.8914046 };

    float ki[] = {
            9.801003e-05, 0.0, 0.0, 0.0, 
            0.0, 0.04217059, 0.0, 0.0 };
    */

    float k[] = {
		0.005961399, 0.0, 0.059124652, 0.0, 
		0.0, 1.0792116, 0.0, 7.8914046 };

    float ki[] = {
		9.801003e-05, 0.0, 0.0, 0.0, 
		0.0, 0.04217059, 0.0, 0.0 };

    controller.init(k, ki, 1.0);
    



    /*
    float a[] = {
		1.0, 0.0, 1.0, 0.0, 
		0.0, 1.0, 0.0, 1.0, 
		0.0, 0.0, 0.9189645, 0.0, 
		0.0, 0.0, 0.0, 0.92 };

    float b[] = {
            0.0, 0.0, 
            0.0, 0.0, 
            0.679863, 0.0, 
            0.0, 0.014874454 };

    float c[] = {
            1.0, 0.0, 0.0, 0.0, 
            0.0, 1.0, 0.0, 0.0, 
            0.0, 0.0, 1.0, 0.0, 
            0.0, 0.0, 0.0, 1.0 };

    float k[] = {
            0.005961399, 0.0, 0.059124652, 0.0, 
            0.0, 1.0792116, 0.0, 7.8914046 };

    float ki[] = {
            9.801003e-05, 0.0, 0.0, 0.0, 
            0.0, 0.04217059, 0.0, 0.0 };

    float f[] = {
            0.3603361, 0.0, 0.13288806, 0.0, 
            0.0, 0.36056542, 0.0, 0.1332538, 
            0.06644403, 0.0, 0.13166018, 0.0, 
            0.0, 0.0666269, 0.0, 0.13192691 };

    controller.init(a, b, k, ki, f, 1.0);
    */

    this->distance_prev = 0.0;
    this->distance      = 0.0;
    this->angle_prev    = 0.0;
    this->angle         = 0.0;


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
    //estimate current state 
    float right_position = motor_control.get_right_position_smooth();
    float left_position  = motor_control.get_left_position_smooth();

    
    this->distance_prev = this->distance;   
    this->distance      = 0.25*(right_position + left_position)*WHEEL_DIAMETER;

    this->angle_prev    = this->angle;
    this->angle         = 0.5*(right_position - left_position)*WHEEL_DIAMETER / WHEEL_BRACE;

    
    // update current state 
    controller.x[0] = this->distance;
    controller.x[1] = this->angle;     
    controller.x[2] = (this->distance  - this->distance_prev);
    controller.x[3] = (this->angle     - this->angle_prev);



    // compute controller output
    controller.step();    

    // send to motors
    float u_forward  = controller.u[0];
    float u_turn     = controller.u[1];    

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
    return controller.x[0];
}

float PositionControl::get_angle()
{
    return controller.x[1];
}

float PositionControl::get_velocity()
{
    return controller.x[2];
}

float PositionControl::get_angular_velocity()
{
    return controller.x[3];
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

