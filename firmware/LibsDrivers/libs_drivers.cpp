#include "libs_drivers.h"



Terminal    terminal;
Timer       timer;    

ADC_driver  adc;

void LibsDriversInit()
{
    // low level init, cache, clock
    drivers_init();  

    // uart initialisation
    uart_init();

    // terminal init
    terminal.init();

    // timer
    timer.init();

    adc.init();
}