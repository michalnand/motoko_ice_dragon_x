#include <libs_drivers.h>
#include "as5600_t.h"

// init magnetic encoders
// print readed raw reading (0 .. 4096) and accumulated angle
void encoder_test()
{
    AS5600T<11, 10, 50, 'C', 'C'> left_encoder;
    AS5600T<5, 12,  50,  'B', 'C'> right_encoder;
    
    left_encoder.init();    
    right_encoder.init();

    uint32_t cnt = 0;
    while (1)
    {
        left_encoder.update();
        right_encoder.update();

        timer.delay_ms(1);

        if ((cnt%100) == 0) 
        {
            terminal << left_encoder.angle << " " << left_encoder.position << " " << right_encoder.angle << " " <<  right_encoder.position << "\n";
        }

        cnt++;
    }
}