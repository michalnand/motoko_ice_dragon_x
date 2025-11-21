#include <libs_drivers.h>



// run both BLDC motors forward, no feedback loop, just simple three phase sine waves
void gyro_test()
{
    TI2C<'C', 2, 3, 50> i2c;
    i2c.init();  

    Gyro gyro;
    

    if (gyro.init(i2c) == 0)
    {
        terminal << "gyro init DONE\n";
    }
    else
    {
        terminal << "gyro init ERROR\n";
    }


    if (gyro.init(i2c) == 0)
    {
        terminal << "gyro init DONE\n";
    }
    else
    {
        terminal << "gyro init ERROR\n";
    }   


    while (1)
    {
        //float res = gyro.read();
        //terminal << res << "\n";

        int32_t res = gyro.read_raw();
        terminal << res << "\n";

        timer.delay_ms(100);
    }
}
