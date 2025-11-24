#include <libs_drivers.h>
#include <adc_driver.h>



#ifdef __cplusplus 
extern "C" {
#endif

ADC_driver *g_adc_driver;
 
void ADC_IRQHandler(void)
{
    if (LL_ADC_IsActiveFlag_EOCS(ADC1))
    {
       

        LL_ADC_ClearFlag_EOCS(ADC1);

        // store result
        g_adc_driver->adc_result[g_adc_driver->adc_current_idx] = LL_ADC_REG_ReadConversionData12(ADC1);

        // next channel
        g_adc_driver->adc_current_idx = (g_adc_driver->adc_current_idx + 1) % ADC_CHANNELS_COUNT;

        // full scan finished
        if (g_adc_driver->adc_current_idx == 0)
        {
            g_adc_driver->callback();
            g_adc_driver->measurement_id++;
        }

        // configure next channel
        LL_ADC_REG_SetSequencerRanks(
            ADC1,
            LL_ADC_REG_RANK_1,
            g_adc_driver->adc_channels[g_adc_driver->adc_current_idx]
        );

        LL_ADC_SetChannelSamplingTime(
            ADC1,
            g_adc_driver->adc_channels[g_adc_driver->adc_current_idx],
            LL_ADC_SAMPLINGTIME_112CYCLES
        );

        // trigger next conversion
        LL_ADC_REG_StartConversionSWStart(ADC1);
    }
} 
 
#ifdef __cplusplus
}
#endif

ADC_driver::ADC_driver()
{

}



void ADC_driver::init()
{
    adc_current_idx = 0;
    measurement_id  = 0;
    g_adc_driver    = this;

    for (uint32_t i = 0; i < ADC_CHANNELS_COUNT; i++)
    {
        adc_result[i] = 0;
    }
    

    Gpio<'A', 0, GPIO_MODE_AN>   adc_0;
    Gpio<'A', 1, GPIO_MODE_AN>   adc_1;
    Gpio<'A', 2, GPIO_MODE_AN>   adc_2;
    Gpio<'A', 3, GPIO_MODE_AN>   adc_3;
    Gpio<'A', 4, GPIO_MODE_AN>   adc_4;
    Gpio<'A', 5, GPIO_MODE_AN>   adc_5;
    Gpio<'A', 6, GPIO_MODE_AN>   adc_6;
    Gpio<'A', 7, GPIO_MODE_AN>   adc_7;

    Gpio<'B', 0, GPIO_MODE_AN>   ir_front_left;
    Gpio<'B', 1, GPIO_MODE_AN>   ir_left;
    Gpio<'C', 0, GPIO_MODE_AN>   ir_right;
    Gpio<'C', 1, GPIO_MODE_AN>   ir_front_right;

    
    adc_channels[0]  = ADC_LINE_0;
    adc_channels[1]  = ADC_LINE_1;
    adc_channels[2]  = ADC_LINE_2;
    adc_channels[3]  = ADC_LINE_3;
    adc_channels[4]  = ADC_LINE_4;
    adc_channels[5]  = ADC_LINE_5;
    adc_channels[6]  = ADC_LINE_6;
    adc_channels[7]  = ADC_LINE_7;

    adc_channels[8]  = ADC_IR_FRONT_RIGHT;
    adc_channels[9]  = ADC_IR_RIGHT;
    adc_channels[10] = ADC_IR_LEFT;
    adc_channels[11] = ADC_IR_FRONT_LEFT;

   
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

    LL_ADC_Disable(ADC1);

    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);

    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, adc_channels[0]);

    LL_ADC_SetChannelSamplingTime(ADC1, adc_channels[0], LL_ADC_SAMPLINGTIME_480CYCLES);

    LL_ADC_EnableIT_EOCS(ADC1); 

    // priority = 3
    NVIC_SetPriority(ADC_IRQn, NVIC_EncodePriority(3, 3, 0));
    NVIC_EnableIRQ(ADC_IRQn);

    LL_ADC_Enable(ADC1);
    while (!LL_ADC_IsEnabled(ADC1))
    {
        __asm("nop");
    }

    LL_ADC_REG_StartConversionSWStart(ADC1);
} 

uint16_t* ADC_driver::get()
{
    return (uint16_t*)adc_result;
}
 
 
void ADC_driver::callback()
{
    ir_sensor.callback();
    line_sensor.callback();
} 