#include <libs_drivers.h>


#include <tests.h>

int main() 
{         
    LibsDriversInit();


    button();

    //adc_test();
    //encoder_test(); 
    //gyro_test();
    
    //motor_pwm_test();
    //motor_driver_test();    

    //motor_identification();
    motor_control_test();
    

    //line_test();
    //ir_test();
    
    /*
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";

    AS5600T<5, 12, 5,  'B', 'C'> right_encoder;
    AS5600T<11, 10, 5, 'C', 'C'> left_encoder;
    
    right_encoder.init();
    left_encoder.init();


    while (1)
    {
        led = 1;
        uint32_t right_angle = right_encoder.read_angle();
        uint32_t left_angle  = left_encoder.read_angle();

        terminal << right_angle << " " << left_angle << "\n";

        led = 0;
        timer.delay_ms(100);
    }
    */


    /*
    button();
  
    timer.delay_ms(500);     
  

    uint32_t start_time = timer.get_time();

    while (1)
    {
        uint32_t curr_time = timer.get_time() - start_time;
        terminal << "time = " << curr_time << "\n";
        led = 1;
        timer.delay_ms(200);
        led = 0;
        timer.delay_ms(800);
    }
    */
    
    return 0;
}
