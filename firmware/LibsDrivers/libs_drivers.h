#ifndef _LIBS_DRIVERS_H_
#define _LIBS_DRIVERS_H_

#include <drivers.h>
#include <common.h>

#include "button.h"
#include "adc_driver.h"
#include "gyro.h"

#include "motor_pwm.h"  


// availible from all files
extern Timer    timer;
extern Terminal terminal;
extern ADC_driver adc;

void LibsDriversInit();

#endif

