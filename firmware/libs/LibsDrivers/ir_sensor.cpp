#include "ir_sensor.h"
#include "drivers.h"
#include "fmath.h"






//cubic polynomial calibration coefficients


//robot A
const float ir_calibration[] = 
{
    -4.03034138e+00,  2.45877538e-01, -1.85604226e-04,  4.91336062e-08,
    2.72571251e+01,  5.72482685e-02, -2.55581094e-05,  6.17147908e-09,
    7.90535532e+00,  1.11825243e-01, -7.09537642e-05,  1.55243437e-08,
    -1.88747631e+01,  2.87659271e-01, -2.23295024e-04,  5.95007835e-08
}; 



void IRSensor::init()
{
    terminal << "ir_sensor init start\n";

    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        ir_off[i]   = 0;
        ir_on[i]    = 0;
        distance[i] = 0;
    }

   
    //initial state
    ir_led          = 0;
    state           = 0;

    filter_coeff    = 0.1;

    measurement_id  = 0;

    terminal << "ir_sensor init [DONE]\n";
}   


void IRSensor::callback()
{
    measurement_id++;

    state++;

    if ((state%10) == 0)
    {
        //read IR leds when turned off
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            ir_off[i] = adc.get()[i + IR_SENSOR_OFFSET];
        }

        //turn on IR led for next step
        ir_led  = 1; 
    }
    else if ((state%10) == 2)
    {
        //read IR leds when turned on
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            ir_on[i] = adc.get()[i + IR_SENSOR_OFFSET];
        }
        
        //turn off IR led for next step
        ir_led  = 0;    


        //compute filters
    
        //if dif is small obstacle is close
        //bigger value, bigger distance
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            //difference
            int dif = 4096 - (ir_off[i] - ir_on[i]);

            //compute distance from raw readings
            float d = calibration((float*)(ir_calibration + i*4), dif);
            
            //float d = dif;      
                
            //filter values
            distance[i] = (1.0 - filter_coeff)*distance[i] + filter_coeff*d;
        }
    }
}

float IRSensor::obstacle_distance()
{
    return min(distance[0], distance[3]);
}   

int IRSensor::obstacle_detected()
{
    float d = obstacle_distance();

    if (d < 80.0)          
    {
        return 2; 
    }
    else if (d < 120.0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
 

float* IRSensor::get()
{
    return distance;
}

float IRSensor::calibration(float *callibration, float x) 
{
    float y; 

    //cubic calibration
    y = callibration[0];
    y+= callibration[1]*x;
    y+= callibration[2]*x*x;
    y+= callibration[3]*x*x*x;

    if (y < 0.0)
    {
        y = 0.0;
    }

    if (y > 200.0)
    {
        y = 200.0;
    }

    return y;
}


void IRSensor::print()
{
    terminal << "ir sensor\n";
    for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
    {
        terminal << ir_sensor.get()[i] << " ";
    }
    terminal << "\n\n\n";
}