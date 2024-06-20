#ifndef _GYRO_H_
#define _GYRO_H_


#include <i2c_interface.h>
 

class Gyro
{
    public:
        int init(I2C_Interface &i2c_interface);

        int32_t  read_raw();
        float read();


    private:
        I2C_Interface *i2c;
        int odr;

        int32_t sensitivity;
        int32_t offset_z;
};


#endif