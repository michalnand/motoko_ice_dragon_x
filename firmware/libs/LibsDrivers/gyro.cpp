#include <gyro.h>
#include <drivers.h>
#include <fmath.h>

#define LSM6DSM_ADDRESS         ((unsigned char)0xD4)
#define WHO_AM_I_VALUE          ((unsigned char)0x6A)
#define GYRO_DPS                ((int32_t)1000)  //250, 500, 1000
//#define GYRO_CALIB              ((float)90.0/78.0)
#define GYRO_CALIB              ((float)180.0/153.0)


#define FUNC_CFG_ACCESS         ((unsigned char)0x01)
#define SENSOR_SYNC_TIME_FRAME  ((unsigned char)0x04)
#define SENSOR_SYNC_RES_RATIO   ((unsigned char)0x05)
#define WHO_AM_I                ((unsigned char)0x0F)


#define CTRL1_XL                ((unsigned char)0x10)
#define CTRL2_G                 ((unsigned char)0x11)
#define CTRL3_C                 ((unsigned char)0x12)
#define CTRL4_C                 ((unsigned char)0x13)
#define CTRL5_C                 ((unsigned char)0x14)
#define CTRL6_C                 ((unsigned char)0x15)
#define CTRL7_G                 ((unsigned char)0x16)
#define CTRL8_XL                ((unsigned char)0x17)
#define CTRL9_XL                ((unsigned char)0x18)
#define CTRL10_C                ((unsigned char)0x19)



#define OUTX_L_G                ((unsigned char)0x22)
#define OUTX_H_G                ((unsigned char)0x23)

#define OUTY_L_G                ((unsigned char)0x24)
#define OUTY_H_G                ((unsigned char)0x25)

#define OUTZ_L_G                ((unsigned char)0x26)
#define OUTZ_H_G                ((unsigned char)0x27)


 
#define OUTX_L_XL               ((unsigned char)0x28)
#define OUTX_H_XL               ((unsigned char)0x29)

#define OUTY_L_XL               ((unsigned char)0x2A)
#define OUTY_H_XL               ((unsigned char)0x2B)

#define OUTZ_L_XL               ((unsigned char)0x2C)
#define OUTZ_H_XL               ((unsigned char)0x2D)




#define    FS_G_245dps          ((unsigned char)0x00)
#define    FS_G_500dps          ((unsigned char)0x04)
#define    FS_G_1000dps         ((unsigned char)0x08)
#define    FS_G_2000dps         ((unsigned char)0x0C)

#define     ODR_G_13Hz          ((unsigned char)0x10)
#define     ODR_G_26Hz          ((unsigned char)0x20)
#define     ODR_G_52Hz          ((unsigned char)0x30)
#define     ODR_G_104Hz         ((unsigned char)0x40)
#define     ODR_G_208Hz         ((unsigned char)0x50)
#define     ODR_G_416Hz         ((unsigned char)0x60)
#define     ODR_G_833Hz         ((unsigned char)0x70)
#define     ODR_G_1660Hz        ((unsigned char)0x80)
#define     ODR_G_3330Hz        ((unsigned char)0x90)
#define     ODR_G_6660Hz        ((unsigned char)0xA0)


#define     G_HM_MODE           ((unsigned char)0x80)
 





int Gyro::init(I2C_Interface &i2c_interface)
{
    terminal << "gyro_sensor init start\n";

    this->i2c               = &i2c_interface;
    this->odr               = 250;

    //software reset
    i2c->write_reg(LSM6DSM_ADDRESS, CTRL3_C,  (1<<0));
    timer.delay_ms(20);
    
    //high performance mode disabled
    i2c->write_reg(LSM6DSM_ADDRESS, CTRL7_G,  0);

    //416Hz output data rate, 1000dps range
    i2c->write_reg(LSM6DSM_ADDRESS, CTRL2_G, FS_G_1000dps | ODR_G_416Hz);

   
     
    timer.delay_ms(50); 

    //check if gyro present
    if (i2c->read_reg(LSM6DSM_ADDRESS, WHO_AM_I) != WHO_AM_I_VALUE)    
    {
        terminal << "gyro init [ERROR]\n";
        return -1;
    } 

    //calibrate for gyro offset
    int32_t calibration_iterations = 100;

    offset_z = 0;
    for (int32_t i = 0; i < calibration_iterations; i++)
    {
        offset_z+= read(); 
        timer.delay_ms(4); 
    } 

    offset_z        = offset_z/calibration_iterations;

    terminal << "gyro_sensor init [DONE]\n";
    return 0;
}



int32_t Gyro::read_raw()
{
    i2c->start();
    i2c->write(LSM6DSM_ADDRESS);
    i2c->write(OUTZ_L_G); 

    i2c->start();
    i2c->write(LSM6DSM_ADDRESS|0x01);

    int16_t z;

    z = ((int16_t)i2c->read(1)) << 0;
    z|= ((int16_t)i2c->read(0)) << 8;

    i2c->stop();

    return z;
}

float Gyro::read()
{
    int32_t raw = read_raw() - offset_z; 

    //convert raw reading into dps
    //convert dps to radians
    return (GYRO_CALIB*raw*GYRO_DPS*2.0*PI)/(32768.0*360.0);
}
