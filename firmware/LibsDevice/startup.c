#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// Memory locations defined by the linker
extern uint32_t _estack[];
extern uint32_t _sdata[], _edata[];
extern uint32_t _etext[];                // End of code/flash
extern void (*__init_array_start)();     //constructors
extern void (*__init_array_end)();

extern int main(void);

// Default interrupt handler
void __attribute__((interrupt("IRQ"))) Default_Handler(void)
{
  
}



void Reset_Handler()                    __attribute__((naked, aligned(2)));
/* Weak definitions of handlers point to Default_Handler if not implemented */
void NMI_Handler()                      __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler()                __attribute__ ((weak, alias("Default_Handler")));
void MemManage_Handler()                __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler()                 __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler()               __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler()                 __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler()                   __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler()                      __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler()                  __attribute__ ((weak, alias("Default_Handler")));

void WWDG_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* Window WatchDog Interrupt                                         */
void PVD_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  /* PVD through EXTI Line detection Interrupt                         */
void TAMP_STAMP_IRQHandler()            __attribute__((weak, alias("Default_Handler")));  /* Tamper and TimeStamp interrupts through the EXTI line             */
void RTC_WKUP_IRQHandler()              __attribute__((weak, alias("Default_Handler")));  /* RTC Wakeup interrupt through the EXTI line                        */
void FLASH_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* FLASH global Interrupt                                            */
void RCC_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  /* RCC global Interrupt                                              */
void EXTI0_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* EXTI Line0 Interrupt                                              */
void EXTI1_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* EXTI Line1 Interrupt                                              */
void EXTI2_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* EXTI Line2 Interrupt                                              */
void EXTI3_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* EXTI Line3 Interrupt                                              */
void EXTI4_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* EXTI Line4 Interrupt                                              */
void DMA1_Stream0_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA1 Stream 0 global Interrupt                                    */
void DMA1_Stream1_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA1 Stream 1 global Interrupt                                    */
void DMA1_Stream2_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA1 Stream 2 global Interrupt                                    */
void DMA1_Stream3_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA1 Stream 3 global Interrupt                                    */
void DMA1_Stream4_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA1 Stream 4 global Interrupt                                    */
void DMA1_Stream5_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA1 Stream 5 global Interrupt                                    */
void DMA1_Stream6_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA1 Stream 6 global Interrupt                                    */
void ADC_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  /* ADC1, ADC2 and ADC3 global Interrupts                             */
void CAN1_TX_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* CAN1 TX Interrupt                                                 */
void CAN1_RX0_IRQHandler()              __attribute__((weak, alias("Default_Handler")));  /* CAN1 RX0 Interrupt                                                */
void CAN1_RX1_IRQHandler()              __attribute__((weak, alias("Default_Handler")));  /* CAN1 RX1 Interrupt                                                */
void CAN1_SCE_IRQHandler()              __attribute__((weak, alias("Default_Handler")));  /* CAN1 SCE Interrupt                                                */
void EXTI9_5_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* External Line[9:5] Interrupts                                     */
void TIM1_BRK_TIM9_IRQHandler()         __attribute__((weak, alias("Default_Handler")));  /* TIM1 Break interrupt and TIM9 global interrupt                    */
void TIM1_UP_TIM10_IRQHandler()         __attribute__((weak, alias("Default_Handler")));  /* TIM1 Update Interrupt and TIM10 global interrupt                  */
void TIM1_TRG_COM_TIM11_IRQHandler()    __attribute__((weak, alias("Default_Handler")));  /* TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
void TIM1_CC_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* TIM1 Capture Compare Interrupt                                    */
void TIM2_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* TIM2 global Interrupt                                             */
void TIM3_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* TIM3 global Interrupt                                             */
void TIM4_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* TIM4 global Interrupt                                             */
void I2C1_EV_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* I2C1 Event Interrupt                                              */
void I2C1_ER_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* I2C1 Error Interrupt                                              */
void I2C2_EV_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* I2C2 Event Interrupt                                              */
void I2C2_ER_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* I2C2 Error Interrupt                                              */
void SPI1_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* SPI1 global Interrupt                                             */
void SPI2_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* SPI2 global Interrupt                                             */
void USART1_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* USART1 global Interrupt                                           */
void USART2_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* USART2 global Interrupt                                           */
void USART3_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* USART3 global Interrupt                                           */
void EXTI15_10_IRQHandler()             __attribute__((weak, alias("Default_Handler")));  /* External Line[15:10] Interrupts                                   */
void RTC_Alarm_IRQHandler()             __attribute__((weak, alias("Default_Handler")));  /* RTC Alarm (A and B) through EXTI Line Interrupt                   */
void OTG_FS_WKUP_IRQHandler()           __attribute__((weak, alias("Default_Handler")));  /* USB OTG FS Wakeup through EXTI line interrupt                     */
void TIM8_BRK_TIM12_IRQHandler()        __attribute__((weak, alias("Default_Handler")));  /* TIM8 Break Interrupt and TIM12 global interrupt                   */
void TIM8_UP_TIM13_IRQHandler()         __attribute__((weak, alias("Default_Handler")));  /* TIM8 Update Interrupt and TIM13 global interrupt                  */
void TIM8_TRG_COM_TIM14_IRQHandler()    __attribute__((weak, alias("Default_Handler")));  /* TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
void TIM8_CC_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* TIM8 Capture Compare Interrupt                                    */
void DMA1_Stream7_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA1 Stream7 Interrupt                                            */
void FMC_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  /* FMC global Interrupt                                              */
void SDMMC1_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* SDMMC1 global Interrupt                                           */
void TIM5_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* TIM5 global Interrupt                                             */
void SPI3_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* SPI3 global Interrupt                                             */
void UART4_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* UART4 global Interrupt                                            */
void UART5_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* UART5 global Interrupt                                            */
void TIM6_DAC_IRQHandler()              __attribute__((weak, alias("Default_Handler")));  /* TIM6 global and DAC1&2 underrun error  interrupts                 */
void TIM7_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* TIM7 global interrupt                                             */
void DMA2_Stream0_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA2 Stream 0 global Interrupt                                    */
void DMA2_Stream1_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA2 Stream 1 global Interrupt                                    */
void DMA2_Stream2_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA2 Stream 2 global Interrupt                                    */
void DMA2_Stream3_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA2 Stream 3 global Interrupt                                    */
void DMA2_Stream4_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA2 Stream 4 global Interrupt                                    */
void ETH_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  /* Ethernet global Interrupt                                         */
void ETH_WKUP_IRQHandler()              __attribute__((weak, alias("Default_Handler")));  /* Ethernet Wakeup through EXTI line Interrupt                       */
void OTG_FS_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* USB OTG FS global Interrupt                                       */
void DMA2_Stream5_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA2 Stream 5 global interrupt                                    */
void DMA2_Stream6_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA2 Stream 6 global interrupt                                    */
void DMA2_Stream7_IRQHandler()          __attribute__((weak, alias("Default_Handler")));  /* DMA2 Stream 7 global interrupt                                    */
void USART6_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* USART6 global interrupt                                           */
void I2C3_EV_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* I2C3 event interrupt                                              */
void I2C3_ER_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* I2C3 error interrupt                                              */
void OTG_HS_EP1_OUT_IRQHandler()        __attribute__((weak, alias("Default_Handler")));  /* USB OTG HS End Point 1 Out global interrupt                       */
void OTG_HS_EP1_IN_IRQHandler()         __attribute__((weak, alias("Default_Handler")));  /* USB OTG HS End Point 1 In global interrupt                        */
void OTG_HS_WKUP_IRQHandler()           __attribute__((weak, alias("Default_Handler")));  /* USB OTG HS Wakeup through EXTI interrupt                          */
void OTG_HS_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* USB OTG HS global interrupt                                       */
void RNG_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  /* RNG global interrupt                                              */
void FPU_IRQHandler()                   __attribute__((weak, alias("Default_Handler")));  /* FPU global interrupt                                              */
void UART7_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* UART7 global interrupt                                            */
void UART8_IRQHandler()                 __attribute__((weak, alias("Default_Handler")));  /* UART8 global interrupt                                            */
void SPI4_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* SPI4 global Interrupt                                             */
void SPI5_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* SPI5 global Interrupt                                             */
void SAI1_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* SAI1 global Interrupt                                             */
void SAI2_IRQHandler()                  __attribute__((weak, alias("Default_Handler")));  /* SAI2 global Interrupt                                             */
void QUADSPI_IRQHandler()               __attribute__((weak, alias("Default_Handler")));  /* Quad SPI global interrupt                                         */
void LPTIM1_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* LP TIM1 interrupt                                                 */
void SDMMC2_IRQHandler()                __attribute__((weak, alias("Default_Handler")));  /* SDMMC2 global Interrupt                                           */


// ----------------------------------------------------------------------------------
// Interrupt vector table (loaded into flash memory at 0x000)
//
void (* const InterruptVector[])()__attribute__ ((section(".isr_vector"), aligned(2))) = {
    (void(*)(void)) (int)_estack,                // Initial stack pointer
    Reset_Handler,                                    // Reset handler
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    /* Interrupts */    
    WWDG_IRQHandler,                /* Window WatchDog Interrupt                                         */
    PVD_IRQHandler,                 /* PVD through EXTI Line detection Interrupt                         */
    TAMP_STAMP_IRQHandler,          /* Tamper and TimeStamp interrupts through the EXTI line             */
    RTC_WKUP_IRQHandler,            /* RTC Wakeup interrupt through the EXTI line                        */
    FLASH_IRQHandler,               /* FLASH global Interrupt                                            */
    RCC_IRQHandler,                 /* RCC global Interrupt                                              */
    EXTI0_IRQHandler,               /* EXTI Line0 Interrupt                                              */
    EXTI1_IRQHandler,               /* EXTI Line1 Interrupt                                              */
    EXTI2_IRQHandler,               /* EXTI Line2 Interrupt                                              */
    EXTI3_IRQHandler,               /* EXTI Line3 Interrupt                                              */
    EXTI4_IRQHandler,               /* EXTI Line4 Interrupt                                              */
    DMA1_Stream0_IRQHandler,        /* DMA1 Stream 0 global Interrupt                                    */
    DMA1_Stream1_IRQHandler,        /* DMA1 Stream 1 global Interrupt                                    */
    DMA1_Stream2_IRQHandler,        /* DMA1 Stream 2 global Interrupt                                    */
    DMA1_Stream3_IRQHandler,        /* DMA1 Stream 3 global Interrupt                                    */
    DMA1_Stream4_IRQHandler,        /* DMA1 Stream 4 global Interrupt                                    */
    DMA1_Stream5_IRQHandler,        /* DMA1 Stream 5 global Interrupt                                    */
    DMA1_Stream6_IRQHandler,        /* DMA1 Stream 6 global Interrupt                                    */
    ADC_IRQHandler,                 /* ADC1, ADC2 and ADC3 global Interrupts                             */
    CAN1_TX_IRQHandler,             /* CAN1 TX Interrupt                                                 */
    CAN1_RX0_IRQHandler,            /* CAN1 RX0 Interrupt                                                */
    CAN1_RX1_IRQHandler,            /* CAN1 RX1 Interrupt                                                */
    CAN1_SCE_IRQHandler,            /* CAN1 SCE Interrupt                                                */
    EXTI9_5_IRQHandler,             /* External Line[9:5] Interrupts                                     */
    TIM1_BRK_TIM9_IRQHandler,       /* TIM1 Break interrupt and TIM9 global interrupt                    */
    TIM1_UP_TIM10_IRQHandler,       /* TIM1 Update Interrupt and TIM10 global interrupt                  */
    TIM1_TRG_COM_TIM11_IRQHandler,  /* TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
    TIM1_CC_IRQHandler,             /* TIM1 Capture Compare Interrupt                                    */
    TIM2_IRQHandler,                /* TIM2 global Interrupt                                             */
    TIM3_IRQHandler,                /* TIM3 global Interrupt                                             */
    TIM4_IRQHandler,                /* TIM4 global Interrupt                                             */
    I2C1_EV_IRQHandler,             /* I2C1 Event Interrupt                                              */
    I2C1_ER_IRQHandler,             /* I2C1 Error Interrupt                                              */
    I2C2_EV_IRQHandler,             /* I2C2 Event Interrupt                                              */
    I2C2_ER_IRQHandler,             /* I2C2 Error Interrupt                                              */
    SPI1_IRQHandler,                /* SPI1 global Interrupt                                             */
    SPI2_IRQHandler,                /* SPI2 global Interrupt                                             */
    USART1_IRQHandler,              /* USART1 global Interrupt                                           */
    USART2_IRQHandler,              /* USART2 global Interrupt                                           */
    USART3_IRQHandler,              /* USART3 global Interrupt                                           */
    EXTI15_10_IRQHandler,           /* External Line[15:10] Interrupts                                   */
    RTC_Alarm_IRQHandler,           /* RTC Alarm (A and B) through EXTI Line Interrupt                   */
    OTG_FS_WKUP_IRQHandler,         /* USB OTG FS Wakeup through EXTI line interrupt                     */
    TIM8_BRK_TIM12_IRQHandler,      /* TIM8 Break Interrupt and TIM12 global interrupt                   */
    TIM8_UP_TIM13_IRQHandler,       /* TIM8 Update Interrupt and TIM13 global interrupt                  */
    TIM8_TRG_COM_TIM14_IRQHandler,  /* TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
    TIM8_CC_IRQHandler,             /* TIM8 Capture Compare Interrupt                                    */
    DMA1_Stream7_IRQHandler,        /* DMA1 Stream7 Interrupt                                            */
    FMC_IRQHandler,                 /* FMC global Interrupt                                              */
    SDMMC1_IRQHandler,              /* SDMMC1 global Interrupt                                           */
    TIM5_IRQHandler,                /* TIM5 global Interrupt                                             */
    SPI3_IRQHandler,                /* SPI3 global Interrupt                                             */
    UART4_IRQHandler,               /* UART4 global Interrupt                                            */
    UART5_IRQHandler,               /* UART5 global Interrupt                                            */
    TIM6_DAC_IRQHandler,            /* TIM6 global and DAC1&2 underrun error  interrupts                 */
    TIM7_IRQHandler,                /* TIM7 global interrupt                                             */
    DMA2_Stream0_IRQHandler,        /* DMA2 Stream 0 global Interrupt                                    */
    DMA2_Stream1_IRQHandler,        /* DMA2 Stream 1 global Interrupt                                    */
    DMA2_Stream2_IRQHandler,        /* DMA2 Stream 2 global Interrupt                                    */
    DMA2_Stream3_IRQHandler,        /* DMA2 Stream 3 global Interrupt                                    */
    DMA2_Stream4_IRQHandler,        /* DMA2 Stream 4 global Interrupt                                    */
    ETH_IRQHandler,                 /* Ethernet global Interrupt                                         */
    ETH_WKUP_IRQHandler,            /* Ethernet Wakeup through EXTI line Interrupt                       */
    OTG_FS_IRQHandler,              /* USB OTG FS global Interrupt                                       */
    DMA2_Stream5_IRQHandler,        /* DMA2 Stream 5 global interrupt                                    */
    DMA2_Stream6_IRQHandler,        /* DMA2 Stream 6 global interrupt                                    */
    DMA2_Stream7_IRQHandler,        /* DMA2 Stream 7 global interrupt                                    */
    USART6_IRQHandler,              /* USART6 global interrupt                                           */
    I2C3_EV_IRQHandler,             /* I2C3 event interrupt                                              */
    I2C3_ER_IRQHandler,             /* I2C3 error interrupt                                              */
    OTG_HS_EP1_OUT_IRQHandler,      /* USB OTG HS End Point 1 Out global interrupt                       */
    OTG_HS_EP1_IN_IRQHandler,       /* USB OTG HS End Point 1 In global interrupt                        */
    OTG_HS_WKUP_IRQHandler,         /* USB OTG HS Wakeup through EXTI interrupt                          */
    0,
    0,
    OTG_HS_IRQHandler,              /* USB OTG HS global interrupt                                       */
    RNG_IRQHandler,                 /* RNG global interrupt                                              */
    FPU_IRQHandler,                 /* FPU global interrupt                                              */
    UART7_IRQHandler,               /* UART7 global interrupt                                            */
    UART8_IRQHandler,               /* UART8 global interrupt                                            */
    SPI4_IRQHandler,                /* SPI4 global Interrupt                                             */
    SPI5_IRQHandler,                /* SPI5 global Interrupt                                             */
    0,
    SAI1_IRQHandler,                /* SAI1 global Interrupt                                             */
    0,
    0,
    0,
    SAI2_IRQHandler,                /* SAI2 global Interrupt                                             */
    QUADSPI_IRQHandler,             /* Quad SPI global interrupt                                         */
    LPTIM1_IRQHandler,              /* LP TIM1 interrupt                                                 */
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    SDMMC2_IRQHandler               /* SDMMC2 global Interrupt                                           */
};

#include "device.h"

void Reset_Handler(void)
{
    // copy values to initialize data segment
    uint32_t *fr        = _etext;
    uint32_t *to        = _sdata;
    unsigned int len    = _edata - _sdata;

    while(len--) 
    {
        *to++ = *fr++;
    }  

    // enable FPU
    SCB->CPACR|= (1<<20)|(1<<21)|(1<<22)|(1<<23);   //full access
    FPU->FPCCR|= ((1<<31)|(1<<30));                 //enable context saving

    //turn on cache
    SCB_EnableICache();
	SCB_EnableDCache();
    
    //call global constructors
    void (**p)() = &__init_array_start;
    for (int i = 0; i < (&__init_array_end - &__init_array_start); i++)
    { 
        p[i]();
    }
    
    main();
}

#ifdef __cplusplus
}
#endif
