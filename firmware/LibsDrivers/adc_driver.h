#ifndef _ADC_DRIVER_H_
#define _ADC_DRIVER_H_ 

#define ADC_CHANNELS_COUNT      ((uint32_t)(8 + 4))

#define LINE_SENSOR_OFFSET      ((uint32_t)(0))
#define IR_SENSOR_OFFSET        ((uint32_t)(8))


//line sensors physical channels
#define ADC_LINE_0                 (LL_ADC_CHANNEL_0)    //PA0   - right sensor
#define ADC_LINE_1                 (LL_ADC_CHANNEL_1)    //PA1
#define ADC_LINE_2                 (LL_ADC_CHANNEL_2)    //PA2
#define ADC_LINE_3                 (LL_ADC_CHANNEL_3)    //PA3
#define ADC_LINE_4                 (LL_ADC_CHANNEL_4)    //PA4
#define ADC_LINE_5                 (LL_ADC_CHANNEL_5)    //PA5
#define ADC_LINE_6                 (LL_ADC_CHANNEL_6)    //PA6
#define ADC_LINE_7                 (LL_ADC_CHANNEL_7)    //PA7   - left sensor

//IR sensors physical channels
#define ADC_IR_FRONT_LEFT           (LL_ADC_CHANNEL_8)    //PB0
#define ADC_IR_LEFT                 (LL_ADC_CHANNEL_9)    //PB1
#define ADC_IR_RIGHT                (LL_ADC_CHANNEL_10)   //PC0
#define ADC_IR_FRONT_RIGHT          (LL_ADC_CHANNEL_11)   //PC1



  
class ADC_driver
{
    public:
        ADC_driver();

        void init();
        uint16_t* get();

        void callback();

    public:    
        uint32_t adc_current_idx;
        uint32_t adc_channels[ADC_CHANNELS_COUNT];
        uint16_t adc_result[ADC_CHANNELS_COUNT];

        uint32_t measurement_id;
};


#endif
