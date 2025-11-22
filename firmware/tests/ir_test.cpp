#include <libs_drivers.h>
#include <tmath.h>

// raw read adc channels
// print raw readings and how many measurements per second
void ir_test()
{
    while (1)
    {
        terminal << "ir sensor\n";
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            terminal << ir_sensor.distance[i] << " ";
        }
        terminal << "\n\n\n";

        timer.delay_ms(200);
    }
}
