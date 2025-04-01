#include <drivers.h>

int main()
{
    drivers_init();
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    // wait for key press
    int key_result = button();

    Gpio<TGPIOB, 2, GPIO_MODE_OUT> led;   
    led = 1;

    while (true)
    {
        led = 1;

        uint32_t m_start = line_sensor.measurement_id;
        timer.delay_ms(100);
        uint32_t m_stop  = line_sensor.measurement_id;

       
        terminal << "ir sensor\n";
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            terminal << ir_sensor.get()[i] << " ";
        }
        terminal << "\n\n"; 

        led = 0;
        timer.delay_ms(100);
    }

    return 0;
}
  