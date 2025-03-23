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

        terminal << "line sensor\n";
        terminal << "line_lost_type  " << line_sensor.line_lost_type << "\n";
        terminal << "left_position   " << line_sensor.left_position << "\n";
        terminal << "right_position  " << line_sensor.right_position << "\n";
        terminal << "sampling rate   " << 10*(m_stop - m_start) << " Hz\n";
        terminal << "\n\n";

        terminal << "ir sensor\n";
        for (unsigned int i = 0; i < IR_SENSORS_COUNT; i++)
        {
            terminal << ir_sensor.get()[i] << " ";
        }
        terminal << "\n\n"; 

        // convert gyro angle from radians to degrees and print
        terminal << "gyro sensor\n";
        terminal << gyro_sensor.read()*(float)(180.0/PI) << "\n";
        terminal << "\n\n";

        // convert wheel angle from radians to degrees and print
        terminal << "encoder sensor\n";
        terminal << "right " << motor_control.get_right_position()*(float)(180.0/PI) << "\n";
        terminal << "left  " << motor_control.get_left_position()*(float)(180.0/PI) << "\n";
        terminal << "\n\n"; 

        terminal << "\n\n";

        led = 0;
        timer.delay_ms(100);
    }

    return 0;
}
  