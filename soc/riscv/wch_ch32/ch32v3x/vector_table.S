/*
 * Copyright (c)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>
#include <zephyr/arch/riscv/csr.h>

GTEXT(_irq_vector_table)
SECTION_FUNC(vector_table, _irq_vector_table)
.p2align 8
    j   __ch32_entry
    .word   0
    j   NMI_Handler                /* NMI */
    j   HardFault_Handler          /* Hard Fault */
    .word  0
    j   Ecall_M_Mode_Handler       /* Ecall M Mode */
    .word   0
    .word   0
    j   Ecall_U_Mode_Handler       /* Ecall U Mode */
    j   Break_Point_Handler        /* Break Point */
    .word   0
    .word   0
    j   SysTick_Handler            /* SysTick */
    .word   0
    j   SW_Handler                 /* SW */
    .word   0

    /* External Interrupts */
    j   WWDG_IRQHandler            /* Window Watchdog */
    j   PVD_IRQHandler             /* PVD through EXTI Line detect */
    j   TAMPER_IRQHandler          /* TAMPER */
    j   RTC_IRQHandler             /* RTC */
    j   FLASH_IRQHandler           /* Flash */
    j   RCC_IRQHandler             /* RCC */
    j   EXTI0_IRQHandler           /* EXTI Line 0 */
    j   EXTI1_IRQHandler           /* EXTI Line 1 */
    j   EXTI2_IRQHandler           /* EXTI Line 2 */
    j   EXTI3_IRQHandler           /* EXTI Line 3 */
    j   EXTI4_IRQHandler           /* EXTI Line 4 */
    j   DMA1_Channel1_IRQHandler   /* DMA1 Channel 1 */
    j   DMA1_Channel2_IRQHandler   /* DMA1 Channel 2 */
    j   DMA1_Channel3_IRQHandler   /* DMA1 Channel 3 */
    j   DMA1_Channel4_IRQHandler   /* DMA1 Channel 4 */
    j   DMA1_Channel5_IRQHandler   /* DMA1 Channel 5 */
    j   DMA1_Channel6_IRQHandler   /* DMA1 Channel 6 */
    j   DMA1_Channel7_IRQHandler   /* DMA1 Channel 7 */
    j   ADC1_2_IRQHandler          /* ADC1_2 */
    j   USB_HP_CAN1_TX_IRQHandler  /* USB HP and CAN1 TX */
    j   USB_LP_CAN1_RX0_IRQHandler /* USB LP and CAN1RX0 */
    j   CAN1_RX1_IRQHandler        /* CAN1 RX1 */
    j   CAN1_SCE_IRQHandler        /* CAN1 SCE */
    j   EXTI9_5_IRQHandler         /* EXTI Line 9..5 */
    j   TIM1_BRK_IRQHandler        /* TIM1 Break */
    j   TIM1_UP_IRQHandler         /* TIM1 Update */
    j   TIM1_TRG_COM_IRQHandler    /* TIM1 Trigger and Commutation */
    j   TIM1_CC_IRQHandler         /* TIM1 Capture Compare */
    j   TIM2_IRQHandler            /* TIM2 */
    j   TIM3_IRQHandler            /* TIM3 */
    j   TIM4_IRQHandler            /* TIM4 */
    j   I2C1_EV_IRQHandler         /* I2C1 Event */
    j   I2C1_ER_IRQHandler         /* I2C1 Error */
    j   I2C2_EV_IRQHandler         /* I2C2 Event */
    j   I2C2_ER_IRQHandler         /* I2C2 Error */
    j   SPI1_IRQHandler            /* SPI1 */
    j   SPI2_IRQHandler            /* SPI2 */
    j   USART1_IRQHandler          /* USART1 */
    j   USART2_IRQHandler          /* USART2 */
    j   USART3_IRQHandler          /* USART3 */
    j   EXTI15_10_IRQHandler       /* EXTI Line 15..10 */
    j   RTCAlarm_IRQHandler        /* RTC Alarm through EXTI Line */
    j   USBWakeUp_IRQHandler       /* USB Wakeup from suspend */
    j   TIM8_BRK_IRQHandler        /* TIM8 Break */
    j   TIM8_UP_IRQHandler         /* TIM8 Update */
    j   TIM8_TRG_COM_IRQHandler    /* TIM8 Trigger and Commutation */
    j   TIM8_CC_IRQHandler         /* TIM8 Capture Compare */
    j   RNG_IRQHandler             /* RNG */
    j   FSMC_IRQHandler            /* FSMC */
    j   SDIO_IRQHandler            /* SDIO */
    j   TIM5_IRQHandler            /* TIM5 */
    j   SPI3_IRQHandler            /* SPI3 */
    j   UART4_IRQHandler           /* UART4 */
    j   UART5_IRQHandler           /* UART5 */
    j   TIM6_IRQHandler            /* TIM6 */
    j   TIM7_IRQHandler            /* TIM7 */
    j   DMA2_Channel1_IRQHandler   /* DMA2 Channel 1 */
    j   DMA2_Channel2_IRQHandler   /* DMA2 Channel 2 */
    j   DMA2_Channel3_IRQHandler   /* DMA2 Channel 3 */
    j   DMA2_Channel4_IRQHandler   /* DMA2 Channel 4 */
    j   DMA2_Channel5_IRQHandler   /* DMA2 Channel 5 */
    j   ETH_IRQHandler             /* ETH */
    j   ETH_WKUP_IRQHandler        /* ETH WakeUp */
    j   CAN2_TX_IRQHandler         /* CAN2 TX */
    j   CAN2_RX0_IRQHandler        /* CAN2 RX0 */
    j   CAN2_RX1_IRQHandler        /* CAN2 RX1 */
    j   CAN2_SCE_IRQHandler        /* CAN2 SCE */
    j   OTG_FS_IRQHandler          /* OTGFS */
    j   USBHSWakeup_IRQHandler     /* USBHS Wakeup */
    j   USBHS_IRQHandler           /* USBHS */
    j   DVP_IRQHandler             /* DVP */
    j   UART6_IRQHandler           /* UART6 */
    j   UART7_IRQHandler           /* UART7 */
    j   UART8_IRQHandler           /* UART8 */
    j   TIM9_BRK_IRQHandler        /* TIM9 Break */
    j   TIM9_UP_IRQHandler         /* TIM9 Update */
    j   TIM9_TRG_COM_IRQHandler    /* TIM9 Trigger and Commutation */
    j   TIM9_CC_IRQHandler         /* TIM9 Capture Compare */
    j   TIM10_BRK_IRQHandler       /* TIM10 Break */
    j   TIM10_UP_IRQHandler        /* TIM10 Update */
    j   TIM10_TRG_COM_IRQHandler   /* TIM10 Trigger and Commutation */
    j   TIM10_CC_IRQHandler        /* TIM10 Capture Compare */
    j   DMA2_Channel6_IRQHandler   /* DMA2 Channel 6 */
    j   DMA2_Channel7_IRQHandler   /* DMA2 Channel 7 */
    j   DMA2_Channel8_IRQHandler   /* DMA2 Channel 8 */
    j   DMA2_Channel9_IRQHandler   /* DMA2 Channel 9 */
    j   DMA2_Channel10_IRQHandler  /* DMA2 Channel 10 */
    j   DMA2_Channel11_IRQHandler  /* DMA2 Channel 11 */

    .weak   NMI_Handler                /* NMI */
NMI_Handler:  1:  j 1b

    .weak   HardFault_Handler          /* Hard Fault */
HardFault_Handler:  1:  j 1b

    .weak   Ecall_M_Mode_Handler       /* Ecall M Mode */
Ecall_M_Mode_Handler:  1:  j 1b

    .weak   Ecall_U_Mode_Handler       /* Ecall U Mode */
Ecall_U_Mode_Handler:  1:  j 1b

    .weak   Break_Point_Handler        /* Break Point */
Break_Point_Handler:  1:  j 1b

    .weak   SysTick_Handler            /* SysTick */
SysTick_Handler:  1:  j 1b

    .weak   SW_Handler                 /* SW */
SW_Handler:  1:  j 1b

    .weak   WWDG_IRQHandler            /* Window Watchdog */
WWDG_IRQHandler:  1:  j 1b

    .weak   PVD_IRQHandler             /* PVD through EXTI Line detect */
PVD_IRQHandler:  1:  j 1b

    .weak   TAMPER_IRQHandler          /* TAMPER */
TAMPER_IRQHandler:  1:  j 1b

    .weak   RTC_IRQHandler             /* RTC */
RTC_IRQHandler:  1:  j 1b

    .weak   FLASH_IRQHandler           /* Flash */
FLASH_IRQHandler:  1:  j 1b

    .weak   RCC_IRQHandler             /* RCC */
RCC_IRQHandler:  1:  j 1b

    .weak   EXTI0_IRQHandler           /* EXTI Line 0 */
EXTI0_IRQHandler:  1:  j 1b

    .weak   EXTI1_IRQHandler           /* EXTI Line 1 */
EXTI1_IRQHandler:  1:  j 1b

    .weak   EXTI2_IRQHandler           /* EXTI Line 2 */
EXTI2_IRQHandler:  1:  j 1b

    .weak   EXTI3_IRQHandler           /* EXTI Line 3 */
EXTI3_IRQHandler:  1:  j 1b

    .weak   EXTI4_IRQHandler           /* EXTI Line 4 */
EXTI4_IRQHandler:  1:  j 1b

    .weak   DMA1_Channel1_IRQHandler   /* DMA1 Channel 1 */
DMA1_Channel1_IRQHandler:  1:  j 1b

    .weak   DMA1_Channel2_IRQHandler   /* DMA1 Channel 2 */
DMA1_Channel2_IRQHandler:  1:  j 1b

    .weak   DMA1_Channel3_IRQHandler   /* DMA1 Channel 3 */
DMA1_Channel3_IRQHandler:  1:  j 1b

    .weak   DMA1_Channel4_IRQHandler   /* DMA1 Channel 4 */
DMA1_Channel4_IRQHandler:  1:  j 1b
        
    .weak   DMA1_Channel5_IRQHandler   /* DMA1 Channel 5 */
DMA1_Channel5_IRQHandler:  1:  j 1b

    .weak   DMA1_Channel6_IRQHandler   /* DMA1 Channel 6 */
DMA1_Channel6_IRQHandler:  1:  j 1b

    .weak   DMA1_Channel7_IRQHandler   /* DMA1 Channel 7 */
DMA1_Channel7_IRQHandler:  1:  j 1b

    .weak   ADC1_2_IRQHandler          /* ADC1_2 */
ADC1_2_IRQHandler:  1:  j 1b

    .weak   USB_HP_CAN1_TX_IRQHandler  /* USB HP and CAN1 TX */
USB_HP_CAN1_TX_IRQHandler:  1:  j 1b

    .weak   USB_LP_CAN1_RX0_IRQHandler /* USB LP and CAN1RX0 */
USB_LP_CAN1_RX0_IRQHandler:  1:  j 1b

    .weak   CAN1_RX1_IRQHandler        /* CAN1 RX1 */
CAN1_RX1_IRQHandler:  1:  j 1b

    .weak   CAN1_SCE_IRQHandler        /* CAN1 SCE */
CAN1_SCE_IRQHandler:  1:  j 1b

    .weak   EXTI9_5_IRQHandler         /* EXTI Line 9..5 */
EXTI9_5_IRQHandler:  1:  j 1b

    .weak   TIM1_BRK_IRQHandler        /* TIM1 Break */
TIM1_BRK_IRQHandler:  1:  j 1b

    .weak   TIM1_UP_IRQHandler         /* TIM1 Update */
TIM1_UP_IRQHandler:  1:  j 1b

    .weak   TIM1_TRG_COM_IRQHandler    /* TIM1 Trigger and Commutation */
TIM1_TRG_COM_IRQHandler:  1:  j 1b
    
    .weak   TIM1_CC_IRQHandler         /* TIM1 Capture Compare */
TIM1_CC_IRQHandler:  1:  j 1b

    .weak   TIM2_IRQHandler            /* TIM2 */
TIM2_IRQHandler:  1:  j 1b

    .weak   TIM3_IRQHandler            /* TIM3 */
TIM3_IRQHandler:  1:  j 1b

    .weak   TIM4_IRQHandler            /* TIM4 */
TIM4_IRQHandler:  1:  j 1b

    .weak   I2C1_EV_IRQHandler         /* I2C1 Event */
I2C1_EV_IRQHandler:  1:  j 1b

    .weak   I2C1_ER_IRQHandler         /* I2C1 Error */
I2C1_ER_IRQHandler:  1:  j 1b
        
    .weak   I2C2_EV_IRQHandler         /* I2C2 Event */
I2C2_EV_IRQHandler:  1:  j 1b

    .weak   I2C2_ER_IRQHandler         /* I2C2 Error */
I2C2_ER_IRQHandler:  1:  j 1b
  
    .weak   SPI1_IRQHandler            /* SPI1 */
SPI1_IRQHandler:  1:  j 1b

    .weak   SPI2_IRQHandler            /* SPI2 */
SPI2_IRQHandler:  1:  j 1b

    .weak   USART1_IRQHandler          /* USART1 */
USART1_IRQHandler:  1:  j 1b
  
    .weak   USART2_IRQHandler          /* USART2 */
USART2_IRQHandler:  1:  j 1b
      
    .weak   USART3_IRQHandler          /* USART3 */
USART3_IRQHandler:  1:  j 1b
   
    .weak   EXTI15_10_IRQHandler       /* EXTI Line 15..10 */
EXTI15_10_IRQHandler:  1:  j 1b
    
    .weak   RTCAlarm_IRQHandler        /* RTC Alarm through EXTI Line */
RTCAlarm_IRQHandler:  1:  j 1b
 
    .weak   USBWakeUp_IRQHandler       /* USB Wakeup from suspend */
USBWakeUp_IRQHandler:  1:  j 1b
   
    .weak   TIM8_BRK_IRQHandler        /* TIM8 Break */
TIM8_BRK_IRQHandler:  1:  j 1b
    
    .weak   TIM8_UP_IRQHandler         /* TIM8 Update */
TIM8_UP_IRQHandler:  1:  j 1b
        
    .weak   TIM8_TRG_COM_IRQHandler    /* TIM8 Trigger and Commutation */
TIM8_TRG_COM_IRQHandler:  1:  j 1b

    .weak   TIM8_CC_IRQHandler         /* TIM8 Capture Compare */
TIM8_CC_IRQHandler:  1:  j 1b
       
    .weak   RNG_IRQHandler             /* RNG */
RNG_IRQHandler:  1:  j 1b
 
    .weak   FSMC_IRQHandler            /* FSMC */
FSMC_IRQHandler:  1:  j 1b

    .weak   SDIO_IRQHandler            /* SDIO */
SDIO_IRQHandler:  1:  j 1b
 
    .weak   TIM5_IRQHandler            /* TIM5 */
TIM5_IRQHandler:  1:  j 1b   

    .weak   SPI3_IRQHandler            /* SPI3 */
SPI3_IRQHandler:  1:  j 1b

    .weak   UART4_IRQHandler           /* UART4 */
UART4_IRQHandler:  1:  j 1b
          
    .weak   UART5_IRQHandler           /* UART5 */
UART5_IRQHandler:  1:  j 1b

    .weak   TIM6_IRQHandler            /* TIM6 */
TIM6_IRQHandler:  1:  j 1b

    .weak   TIM7_IRQHandler            /* TIM7 */
TIM7_IRQHandler:  1:  j 1b            

    .weak   DMA2_Channel1_IRQHandler   /* DMA2 Channel 1 */
DMA2_Channel1_IRQHandler:  1:  j 1b

    .weak   DMA2_Channel2_IRQHandler   /* DMA2 Channel 2 */
DMA2_Channel2_IRQHandler:  1:  j 1b

    .weak   DMA2_Channel3_IRQHandler   /* DMA2 Channel 3 */
DMA2_Channel3_IRQHandler:  1:  j 1b
   
    .weak   DMA2_Channel4_IRQHandler   /* DMA2 Channel 4 */
DMA2_Channel4_IRQHandler:  1:  j 1b

    .weak   DMA2_Channel5_IRQHandler   /* DMA2 Channel 5 */
DMA2_Channel5_IRQHandler:  1:  j 1b

    .weak   ETH_IRQHandler             /* ETH */
ETH_IRQHandler:  1:  j 1b
     
    .weak   ETH_WKUP_IRQHandler        /* ETH WakeUp */
ETH_WKUP_IRQHandler:  1:  j 1b
   
    .weak   CAN2_TX_IRQHandler         /* CAN2 TX */
CAN2_TX_IRQHandler:  1:  j 1b
         
    .weak   CAN2_RX0_IRQHandler        /* CAN2 RX0 */
CAN2_RX0_IRQHandler:  1:  j 1b
 
    .weak   CAN2_RX1_IRQHandler        /* CAN2 RX1 */
CAN2_RX1_IRQHandler:  1:  j 1b
      
    .weak   CAN2_SCE_IRQHandler        /* CAN2 SCE */
CAN2_SCE_IRQHandler:  1:  j 1b
    
    .weak   OTG_FS_IRQHandler          /* OTGFS */
OTG_FS_IRQHandler:  1:  j 1b
 
    .weak   USBHSWakeup_IRQHandler     /* USBHS Wakeup */
USBHSWakeup_IRQHandler:  1:  j 1b
   
    .weak   USBHS_IRQHandler           /* USBHS */
USBHS_IRQHandler:  1:  j 1b
      
    .weak   DVP_IRQHandler             /* DVP */
DVP_IRQHandler:  1:  j 1b

    .weak   UART6_IRQHandler           /* UART6 */
UART6_IRQHandler:  1:  j 1b
      
    .weak   UART7_IRQHandler           /* UART7 */
UART7_IRQHandler:  1:  j 1b

    .weak   UART8_IRQHandler           /* UART8 */
UART8_IRQHandler:  1:  j 1b

    .weak   TIM9_BRK_IRQHandler        /* TIM9 Break */
TIM9_BRK_IRQHandler:  1:  j 1b

    .weak   TIM9_UP_IRQHandler         /* TIM9 Update */
TIM9_UP_IRQHandler:  1:  j 1b

    .weak   TIM9_TRG_COM_IRQHandler    /* TIM9 Trigger and Commutation */
TIM9_TRG_COM_IRQHandler:  1:  j 1b

    .weak   TIM9_CC_IRQHandler         /* TIM9 Capture Compare */
TIM9_CC_IRQHandler:  1:  j 1b

    .weak   TIM10_BRK_IRQHandler       /* TIM10 Break */
TIM10_BRK_IRQHandler:  1:  j 1b

    .weak   TIM10_UP_IRQHandler        /* TIM10 Update */
TIM10_UP_IRQHandler:  1:  j 1b

    .weak   TIM10_TRG_COM_IRQHandler   /* TIM10 Trigger and Commutation */
TIM10_TRG_COM_IRQHandler:  1:  j 1b

    .weak   TIM10_CC_IRQHandler        /* TIM10 Capture Compare */
TIM10_CC_IRQHandler:  1:  j 1b   

    .weak   DMA2_Channel6_IRQHandler   /* DMA2 Channel 6 */
DMA2_Channel6_IRQHandler:  1:  j 1b

    .weak   DMA2_Channel7_IRQHandler   /* DMA2 Channel 7 */
DMA2_Channel7_IRQHandler:  1:  j 1b

    .weak   DMA2_Channel8_IRQHandler   /* DMA2 Channel 8 */
DMA2_Channel8_IRQHandler:  1:  j 1b

    .weak   DMA2_Channel9_IRQHandler   /* DMA2 Channel 9 */
DMA2_Channel9_IRQHandler:  1:  j 1b

    .weak   DMA2_Channel10_IRQHandler  /* DMA2 Channel 10 */
DMA2_Channel10_IRQHandler:  1:  j 1b

    .weak   DMA2_Channel11_IRQHandler  /* DMA2 Channel 11 */
DMA2_Channel11_IRQHandler:  1:  j 1b
