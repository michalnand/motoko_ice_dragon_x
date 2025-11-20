#include "libs_drivers.h"



Terminal    terminal;
Timer       timer;    

void LibsDriversInit()
{
    // low level init, cache, clock
    drivers_init();  

    uart_init();

    timer.init();

}