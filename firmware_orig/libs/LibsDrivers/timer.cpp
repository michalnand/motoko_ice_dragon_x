#include "timer.h"
#include <device.h>


volatile uint32_t g_time = 0;

#ifdef __cplusplus
extern "C" {
#endif

void SysTick_Handler(void)
{
    g_time++;
}

#ifdef __cplusplus
}
#endif


Timer::Timer()
{
    
} 

void Timer::init(uint32_t frequency)
{
    g_time      = 0;

    //interrupt every 1ms 
    SysTick_Config(SystemCoreClock/frequency);
    __enable_irq();

    this->delay_ms(100);
}

void Timer::delay_ms(uint32_t time_ms)
{
    time_ms = time_ms + g_time;
    while (time_ms > g_time)
    {
        __asm("wfi");
    }
}

uint32_t Timer::get_time()
{
    volatile uint32_t result = g_time;

    return result;
}

// returns nanoseconds resolution time
uint64_t Timer::get_ns_time()
{
    __disable_irq();
    //holds coarse resolution
    uint64_t coarse_result = (uint64_t)1000000*(uint64_t)g_time;
    //holds fine resolution
    uint64_t fine_result   = SysTick->LOAD - SysTick->VAL;
    __enable_irq(); 

    //convert raw counter value into ns
    fine_result = (fine_result*(uint64_t)1000000000)/SystemCoreClock;

    return coarse_result + fine_result;
}
