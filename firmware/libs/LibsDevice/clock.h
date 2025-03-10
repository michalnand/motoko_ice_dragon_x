#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stm32f7xx.h>

#define SysClok216_8HSE    ((uint32_t)0)
#define SysClok312_8HSE    ((uint32_t)1)
#define SysClok216_24HSE   ((uint32_t)2)
#define SysClok312_24HSE   ((uint32_t)3)
 
extern uint32_t APB1Clock;

void SetSysClock(uint32_t mode = SysClok216_8HSE);


inline uint32_t timer_period(uint32_t dt_us)
{
    return ((APB1Clock/1000000)*dt_us) + 1;
}

#endif
