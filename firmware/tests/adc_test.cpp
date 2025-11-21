#include <libs_drivers.h>

// raw read adc channels
// print raw readings and how many measurements per second
void adc_test()
{
    // turn on line led
    Gpio<'C', 4, GPIO_MODE_OUT> led;
    led = 1; 


    while (1)
    {
        uint32_t counter_prev = adc.measurement_id;
        timer.delay_ms(100);
        uint32_t counter_now = adc.measurement_id;

        uint32_t fps = (counter_now - counter_prev)*10;

        terminal << "fps = " << fps << "\n";

        for (unsigned int i = 0; i < ADC_CHANNELS_COUNT; i++)
        {
            uint32_t adc_res = adc.get()[i];
            terminal << adc_res << " ";
        }

        terminal << "\n\n";
    }
}
