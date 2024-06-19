#include "pwm.h"
#include <drivers.h>

void PWMLeft::init()
{
    //left motor PWM controll pins 
    //PWMA      : PC6, TIM3_CH1
    //PWMB      : PC7, TIM3_CH2
    
    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 

    //init pins
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Medium_Speed;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    //alternating functions for pins 
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF2);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF2);
    

    //init timer 3
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

    //enable clock for timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
     
    // PWM_frequency = timer_clk / (TIM_Period + 1)
    // TIM_Period = timer_clk / PWM_frequency - 1
    TIM_BaseStruct.TIM_Prescaler    = 0;
    TIM_BaseStruct.TIM_CounterMode  = TIM_CounterMode_Up;

    TIM_BaseStruct.TIM_Period               = PWM_PERIOD;
    TIM_BaseStruct.TIM_ClockDivision        = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter    = 0;
	 
    // Initialize TIM3
    TIM_TimeBaseInit(TIM3, &TIM_BaseStruct);
    TIM_Cmd(TIM3, ENABLE);  
    


    //PWM output settings
    TIM_OCInitTypeDef TIM_OCStruct;
    
    // PWM mode 2 : clear on compare match
    TIM_OCStruct.TIM_OCMode         = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState    = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity     = TIM_OCPolarity_Low;

    //PC6, TIM3_CH1
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC1Init(TIM3, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    //PC7, TIM3_CH2
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC2Init(TIM3, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void PWMLeft::set(int32_t pwm)
{
    if (pwm > ((int32_t)PWM_PERIOD-1))
    {
        pwm = ((int32_t)PWM_PERIOD-1);
    }

    if (pwm < -((int32_t)PWM_PERIOD-1))
    {
        pwm = -((int32_t)PWM_PERIOD-1);
    }  

    if (pwm > 0)
    {
        TIM3->CCR1 = PWM_PERIOD - pwm;
        TIM3->CCR2 = PWM_PERIOD;      
    }   
    else if (pwm < 0)     
    {
        pwm = -pwm;     
        
        TIM3->CCR1 = PWM_PERIOD;      
        TIM3->CCR2 = PWM_PERIOD - pwm;
    }
    else
    {
        //brake mode
        TIM3->CCR1 = PWM_PERIOD;
        TIM3->CCR2 = PWM_PERIOD;
    }
}







void PWMRight::init()
{
    //right motor PWM controll pins 
    //PWMA      : PB6, TIM4_CH1
    //PWMB      : PB7, TIM4_CH2

    GPIO_InitTypeDef GPIO_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 

    //init pins
    GPIO_InitStruct.GPIO_Pin    = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_OType  = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd   = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Mode   = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed  = GPIO_Medium_Speed;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    //alternating functions for pins 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF2);
 

    //init timer 4
    TIM_TimeBaseInitTypeDef TIM_BaseStruct;

    //enable clock for timer
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
     
    // PWM_frequency = timer_clk / (TIM_Period + 1)
    // TIM_Period = timer_clk / PWM_frequency - 1
    TIM_BaseStruct.TIM_Prescaler    = 0;
    TIM_BaseStruct.TIM_CounterMode  = TIM_CounterMode_Up;

    TIM_BaseStruct.TIM_Period               = PWM_PERIOD;
    TIM_BaseStruct.TIM_ClockDivision        = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter    = 0;
	 
    // Initialize TIM4
    TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
    TIM_Cmd(TIM4, ENABLE);
        


    //PWM output settings
    TIM_OCInitTypeDef TIM_OCStruct;
    
    // PWM mode 2 : clear on compare match
    // PWM mode 1 : set on compare match 
    TIM_OCStruct.TIM_OCMode         = TIM_OCMode_PWM2;
    TIM_OCStruct.TIM_OutputState    = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity     = TIM_OCPolarity_Low;

    //PB6, TIM4_CH1
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC1Init(TIM4, &TIM_OCStruct);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    //PB7, TIM4_CH2
    TIM_OCStruct.TIM_Pulse = 0;
    TIM_OC2Init(TIM4, &TIM_OCStruct);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void PWMRight::set(int32_t pwm)
{
    if (pwm > ((int32_t)PWM_PERIOD-1))
    {
        pwm = ((int32_t)PWM_PERIOD-1);
    }

    if (pwm < -((int32_t)PWM_PERIOD-1))
    {
        pwm = -((int32_t)PWM_PERIOD-1);
    }  

    if (pwm > 0)
    {
        TIM4->CCR1 = PWM_PERIOD;
        TIM4->CCR2 = PWM_PERIOD - pwm;
    }   
    else if (pwm < 0)     
    {
        pwm = -pwm;     
        
        TIM4->CCR1 = PWM_PERIOD - pwm;      
        TIM4->CCR2 = PWM_PERIOD;
    }
    else
    {
        //brake mode
        TIM4->CCR1 = PWM_PERIOD;
        TIM4->CCR2 = PWM_PERIOD;
    }
}