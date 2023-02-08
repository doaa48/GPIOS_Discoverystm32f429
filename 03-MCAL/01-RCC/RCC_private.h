/********************************************************************************
 *                                                                              *
 * [FILE NAME]:   RCC_private.h                                                 *
 *                                                                              *
 * [AUTHOR]:      Ahmed Saeed                                                   *
 *                                                                              *
 * [DATE]:        06/2/2023                                                     *
 *                                                                              *
 * [DESCRIPTION]: Header file for the RCC driver Registers                      *
 *                                                                              *
 * [VERSION]:     1.0.0                                                         *
 *                                                                              *
 *******************************************************************************/

#ifndef RCC_PRIVATE_H_
#define RCC_PRIVATE_H_

/*
 * Description :
 * Configuration of the Cortex-M4 Processor and Core Peripherals.
 */
//#define __CM4_REV                 0x0001U  /*!< Core revision r0p1                            */
//#define __MPU_PRESENT             1U       /*!< STM32F4XX provides an MPU                     */
//#define __NVIC_PRIO_BITS          4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
//#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used  */
//#define __FPU_PRESENT             1U       /* FPU present */




/*
 * Description :
 * STM32F4XX Interrupt Number Definition, according to the selected device.
 */
//typedef enum
//{
///******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
//  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
//  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
//  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
//  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
//  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
//  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
//  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
//  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
///******  STM32 specific Interrupt Numbers **********************************************************************/
//  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
//  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
//  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
//  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
//  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
//  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
//  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
//  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
//  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
//  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
//  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
//  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
//  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
//  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
//  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
//  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
//  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
//  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
//  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
//  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
//  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
//  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
//  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
//  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
//  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
//  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
//  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
//  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
//  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
//  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
//  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
//  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
//  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
//  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
//  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
//  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
//  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
//  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
//  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
//  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
//  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
//  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
//  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
//  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
//  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
//  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
//  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
//  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
//  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
//  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
//  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
//  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
//  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
//  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
//  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
//  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
//  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
//  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
//  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
//  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
//  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
//  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
//  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
//  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
//  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
//  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
//  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
//  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
//  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
//  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
//  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
//  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
//  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
//  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
//  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
//  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
//  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
//  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
//  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
//  HASH_RNG_IRQn               = 80,     /*!< Hash and Rng global interrupt                                     */
//  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
//  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
//  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
//  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
//  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
//  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
//  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
//  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                              */
//  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                        */
//  DMA2D_IRQn                  = 90      /*!< DMA2D global Interrupt                                            */
//} IRQn_Type;



//#include "core_cm4.h"             /* Cortex-M4 processor and core peripherals */
//#include "system_stm32f4xx.h"
#include "std_types.h"



/*
 * Description :
 * Reset and Clock Control Registers.
 */
typedef struct
{
  volatile uint32 CR;            /* RCC clock control register,                                  Address offset: 0x00 */
  volatile uint32 PLLCFGR;       /* RCC PLL configuration register,                              Address offset: 0x04 */
  volatile uint32 CFGR;          /* RCC clock configuration register,                            Address offset: 0x08 */
  volatile uint32 CIR;           /* RCC clock interrupt register,                                Address offset: 0x0C */
  volatile uint32 AHB1RSTR;      /* RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  volatile uint32 AHB2RSTR;      /* RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  volatile uint32 AHB3RSTR;      /* RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32      RESERVED0;         /* Reserved, 0x1C                                                                    */
  volatile uint32 APB1RSTR;      /* RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  volatile uint32 APB2RSTR;      /* RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32      RESERVED1[2];      /* Reserved, 0x28-0x2C                                                               */
  volatile uint32 AHB1ENR;       /* RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  volatile uint32 AHB2ENR;       /* RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  volatile uint32 AHB3ENR;       /* RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32      RESERVED2;         /* Reserved, 0x3C                                                                    */
  volatile uint32 APB1ENR;       /* RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  volatile uint32 APB2ENR;       /* RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32      RESERVED3[2];      /* Reserved, 0x48-0x4C                                                               */
  volatile uint32 AHB1LPENR;     /* RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  volatile uint32 AHB2LPENR;     /* RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  volatile uint32 AHB3LPENR;     /* RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32      RESERVED4;         /* Reserved, 0x5C                                                                    */
  volatile uint32 APB1LPENR;     /* RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  volatile uint32 APB2LPENR;     /* RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32      RESERVED5[2];      /* Reserved, 0x68-0x6C                                                               */
  volatile uint32 BDCR;          /* RCC Backup domain control register,                          Address offset: 0x70 */
  volatile uint32 CSR;           /* RCC clock control & status register,                         Address offset: 0x74 */
  uint32      RESERVED6[2];      /* Reserved, 0x78-0x7C                                                               */
  volatile uint32 SSCGR;         /* RCC spread spectrum clock generation register,               Address offset: 0x80 */
  volatile uint32 PLLI2SCFGR;    /* RCC PLLI2S configuration register,                           Address offset: 0x84 */
  volatile uint32 PLLSAICFGR;    /* RCC PLLSAI configuration register,                           Address offset: 0x88 */
  volatile uint32 DCKCFGR;       /* RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_TypeDef;


/*
 * Description :
 * FLASH Registers.
 */
typedef struct
{
  volatile uint32 ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  volatile uint32 KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  volatile uint32 OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  volatile uint32 SR;       /*!< FLASH status register,           Address offset: 0x0C */
  volatile uint32 CR;       /*!< FLASH control register,          Address offset: 0x10 */
  volatile uint32 OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  volatile uint32 OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;

/********************  Bit definition for FLASH_ACR register  ********************/
#define FLASH_ACR_LATENCY_Pos          (0U)
#define FLASH_ACR_LATENCY              (0xFUL << FLASH_ACR_LATENCY_Pos)         /*!< 0x0000000F */


/*
 * Description :
 * Power Control Registers.
 */
typedef struct
{
  volatile uint32 CR;   /*!< PWR power control register,        Address offset: 0x00 */
  volatile uint32 CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;


/********************  Bit definition for PWR_CR register  ********************/
#define PWR_CR_DBP_Pos         (8U)                            /*!< Disable Backup Domain write protection */
#define PWR_CR_DBP             (0x1UL << PWR_CR_DBP_Pos)                        /*!< 0x00000100 */




#define PERIPH_BASE           0x40000000UL /* Peripheral base address in the alias region */

/* Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)


#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)


#define PWR                   ((PWR_TypeDef *) PWR_BASE)
#define FLASH                 ((FLASH_TypeDef *) FLASH_R_BASE)
#define RCC                   ((RCC_TypeDef *) RCC_BASE)







/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos                   (0U)
#define RCC_CR_HSION                       (0x1UL << RCC_CR_HSION_Pos)          /*!< 0x00000001 */

#define RCC_CR_HSIRDY_Pos                  (1U)
#define RCC_CR_HSIRDY                      (0x1UL << RCC_CR_HSIRDY_Pos)         /*!< 0x00000002 */

#define RCC_CR_HSITRIM_Pos                 (3U)
#define RCC_CR_HSITRIM                     (0x1FUL << RCC_CR_HSITRIM_Pos)       /*!< 0x000000F8 */

#define RCC_CR_HSICAL_Pos                  (8U)
#define RCC_CR_HSICAL                      (0xFFUL << RCC_CR_HSICAL_Pos)        /*!< 0x0000FF00 */


#define RCC_CR_HSEON_Pos                   (16U)
#define RCC_CR_HSEON                       (0x1UL << RCC_CR_HSEON_Pos)          /*!< 0x00010000 */

#define RCC_CR_HSERDY_Pos                  (17U)
#define RCC_CR_HSERDY                      (0x1UL << RCC_CR_HSERDY_Pos)         /*!< 0x00020000 */

#define RCC_CR_HSEBYP_Pos                  (18U)
#define RCC_CR_HSEBYP                      (0x1UL << RCC_CR_HSEBYP_Pos)         /*!< 0x00040000 */

#define RCC_CR_CSSON_Pos                   (19U)
#define RCC_CR_CSSON                       (0x1UL << RCC_CR_CSSON_Pos)          /*!< 0x00080000 */

#define RCC_CR_PLLON_Pos                   (24U)
#define RCC_CR_PLLON                       (0x1UL << RCC_CR_PLLON_Pos)          /*!< 0x01000000 */

#define RCC_CR_PLLRDY_Pos                  (25U)
#define RCC_CR_PLLRDY                      (0x1UL << RCC_CR_PLLRDY_Pos)         /*!< 0x02000000 */

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_PLLI2S_SUPPORT                                                     /*!< Support PLLI2S oscillator */

#define RCC_CR_PLLI2SON_Pos                (26U)
#define RCC_CR_PLLI2SON                    (0x1UL << RCC_CR_PLLI2SON_Pos)       /*!< 0x04000000 */

#define RCC_CR_PLLI2SRDY_Pos               (27U)
#define RCC_CR_PLLI2SRDY                   (0x1UL << RCC_CR_PLLI2SRDY_Pos)      /*!< 0x08000000 */

/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_PLLSAI_SUPPORT                                                     /*!< Support PLLSAI oscillator */

#define RCC_CR_PLLSAION_Pos                (28U)
#define RCC_CR_PLLSAION                    (0x1UL << RCC_CR_PLLSAION_Pos)       /*!< 0x10000000 */

#define RCC_CR_PLLSAIRDY_Pos               (29U)
#define RCC_CR_PLLSAIRDY                   (0x1UL << RCC_CR_PLLSAIRDY_Pos)      /*!< 0x20000000 */

/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLM_Pos               (0U)
#define RCC_PLLCFGR_PLLM                   (0x3FUL << RCC_PLLCFGR_PLLM_Pos)     /*!< 0x0000003F */

#define RCC_PLLCFGR_PLLN_Pos               (6U)
#define RCC_PLLCFGR_PLLN                   (0x1FFUL << RCC_PLLCFGR_PLLN_Pos)    /*!< 0x00007FC0 */

#define RCC_PLLCFGR_PLLP_Pos               (16U)
#define RCC_PLLCFGR_PLLP                   (0x3UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00030000 */
#define RCC_PLLCFGR_PLLP_0                 (0x1UL << RCC_PLLCFGR_PLLP_Pos)      /*!< 0x00010000 */

#define RCC_PLLCFGR_PLLSRC_Pos             (22U)
#define RCC_PLLCFGR_PLLSRC                 (0x1UL << RCC_PLLCFGR_PLLSRC_Pos)    /*!< 0x00400000 */

#define RCC_PLLCFGR_PLLQ_Pos               (24U)
#define RCC_PLLCFGR_PLLQ                   (0xFUL << RCC_PLLCFGR_PLLQ_Pos)      /*!< 0x0F000000 */


/********************  Bit definition for RCC_CFGR register  ******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                    (0U)                                 /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW                        (0x3UL << RCC_CFGR_SW_Pos)           /*!< 0x00000003 */

#define RCC_CFGR_SW_HSI                    0x00000000U                         /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                    0x00000001U                         /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                    0x00000002U                         /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                   (2U)                                 /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS                       (0x3UL << RCC_CFGR_SWS_Pos)          /*!< 0x0000000C */

#define RCC_CFGR_SWS_HSI                   0x00000000U                         /*!< HSI oscillator used as system clock        */
#define RCC_CFGR_SWS_HSE                   0x00000004U                         /*!< HSE oscillator used as system clock        */
#define RCC_CFGR_SWS_PLL                   0x00000008U                         /*!< PLL used as system clock                   */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                  (4U)                                 /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE                      (0xFUL << RCC_CFGR_HPRE_Pos)         /*!< 0x000000F0 */

#define RCC_CFGR_HPRE_DIV1                 0x00000000U                         /*!< SYSCLK not divided    */
#define RCC_CFGR_HPRE_DIV2                 0x00000080U                         /*!< SYSCLK divided by 2   */
#define RCC_CFGR_HPRE_DIV4                 0x00000090U                         /*!< SYSCLK divided by 4   */
#define RCC_CFGR_HPRE_DIV8                 0x000000A0U                         /*!< SYSCLK divided by 8   */
#define RCC_CFGR_HPRE_DIV16                0x000000B0U                         /*!< SYSCLK divided by 16  */
#define RCC_CFGR_HPRE_DIV64                0x000000C0U                         /*!< SYSCLK divided by 64  */
#define RCC_CFGR_HPRE_DIV128               0x000000D0U                         /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256               0x000000E0U                         /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512               0x000000F0U                         /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                 (10U)                                /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1                     (0x7UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */

#define RCC_CFGR_PPRE1_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE1_DIV2                0x00001000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE1_DIV4                0x00001400U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE1_DIV8                0x00001800U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE1_DIV16               0x00001C00U                         /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                 (13U)                                /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2                     (0x7UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */

#define RCC_CFGR_PPRE2_DIV1                0x00000000U                         /*!< HCLK not divided   */
#define RCC_CFGR_PPRE2_DIV2                0x00008000U                         /*!< HCLK divided by 2  */
#define RCC_CFGR_PPRE2_DIV4                0x0000A000U                         /*!< HCLK divided by 4  */
#define RCC_CFGR_PPRE2_DIV8                0x0000C000U                         /*!< HCLK divided by 8  */
#define RCC_CFGR_PPRE2_DIV16               0x0000E000U                         /*!< HCLK divided by 16 */

/*!< RTCPRE configuration */
#define RCC_CFGR_RTCPRE_Pos                (16U)
#define RCC_CFGR_RTCPRE                    (0x1FUL << RCC_CFGR_RTCPRE_Pos)      /*!< 0x001F0000 */

/*!< MCO1 configuration */
#define RCC_CFGR_MCO1_Pos                  (21U)
#define RCC_CFGR_MCO1                      (0x3UL << RCC_CFGR_MCO1_Pos)         /*!< 0x00600000 */

#define RCC_CFGR_I2SSRC_Pos                (23U)
#define RCC_CFGR_I2SSRC                    (0x1UL << RCC_CFGR_I2SSRC_Pos)       /*!< 0x00800000 */

#define RCC_CFGR_MCO1PRE_Pos               (24U)
#define RCC_CFGR_MCO1PRE                   (0x7UL << RCC_CFGR_MCO1PRE_Pos)      /*!< 0x07000000 */

#define RCC_CFGR_MCO2PRE_Pos               (27U)
#define RCC_CFGR_MCO2PRE                   (0x7UL << RCC_CFGR_MCO2PRE_Pos)      /*!< 0x38000000 */

#define RCC_CFGR_MCO2_Pos                  (30U)
#define RCC_CFGR_MCO2                      (0x3UL << RCC_CFGR_MCO2_Pos)         /*!< 0xC0000000 */

/********************  Bit definition for RCC_CIR register  *******************/
#define RCC_CIR_LSIRDYF_Pos                (0U)
#define RCC_CIR_LSIRDYF                    (0x1UL << RCC_CIR_LSIRDYF_Pos)       /*!< 0x00000001 */

#define RCC_CIR_LSERDYF_Pos                (1U)
#define RCC_CIR_LSERDYF                    (0x1UL << RCC_CIR_LSERDYF_Pos)       /*!< 0x00000002 */

#define RCC_CIR_HSIRDYF_Pos                (2U)
#define RCC_CIR_HSIRDYF                    (0x1UL << RCC_CIR_HSIRDYF_Pos)       /*!< 0x00000004 */

#define RCC_CIR_HSERDYF_Pos                (3U)
#define RCC_CIR_HSERDYF                    (0x1UL << RCC_CIR_HSERDYF_Pos)       /*!< 0x00000008 */

#define RCC_CIR_PLLRDYF_Pos                (4U)
#define RCC_CIR_PLLRDYF                    (0x1UL << RCC_CIR_PLLRDYF_Pos)       /*!< 0x00000010 */

#define RCC_CIR_PLLI2SRDYF_Pos             (5U)
#define RCC_CIR_PLLI2SRDYF                 (0x1UL << RCC_CIR_PLLI2SRDYF_Pos)    /*!< 0x00000020 */

#define RCC_CIR_PLLSAIRDYF_Pos             (6U)
#define RCC_CIR_PLLSAIRDYF                 (0x1UL << RCC_CIR_PLLSAIRDYF_Pos)    /*!< 0x00000040 */

#define RCC_CIR_CSSF_Pos                   (7U)
#define RCC_CIR_CSSF                       (0x1UL << RCC_CIR_CSSF_Pos)          /*!< 0x00000080 */

#define RCC_CIR_LSIRDYIE_Pos               (8U)
#define RCC_CIR_LSIRDYIE                   (0x1UL << RCC_CIR_LSIRDYIE_Pos)      /*!< 0x00000100 */

#define RCC_CIR_LSERDYIE_Pos               (9U)
#define RCC_CIR_LSERDYIE                   (0x1UL << RCC_CIR_LSERDYIE_Pos)      /*!< 0x00000200 */

#define RCC_CIR_HSIRDYIE_Pos               (10U)
#define RCC_CIR_HSIRDYIE                   (0x1UL << RCC_CIR_HSIRDYIE_Pos)      /*!< 0x00000400 */

#define RCC_CIR_HSERDYIE_Pos               (11U)
#define RCC_CIR_HSERDYIE                   (0x1UL << RCC_CIR_HSERDYIE_Pos)      /*!< 0x00000800 */

#define RCC_CIR_PLLRDYIE_Pos               (12U)
#define RCC_CIR_PLLRDYIE                   (0x1UL << RCC_CIR_PLLRDYIE_Pos)      /*!< 0x00001000 */

#define RCC_CIR_PLLI2SRDYIE_Pos            (13U)
#define RCC_CIR_PLLI2SRDYIE                (0x1UL << RCC_CIR_PLLI2SRDYIE_Pos)   /*!< 0x00002000 */

#define RCC_CIR_PLLSAIRDYIE_Pos            (14U)
#define RCC_CIR_PLLSAIRDYIE                (0x1UL << RCC_CIR_PLLSAIRDYIE_Pos)   /*!< 0x00004000 */

#define RCC_CIR_LSIRDYC_Pos                (16U)
#define RCC_CIR_LSIRDYC                    (0x1UL << RCC_CIR_LSIRDYC_Pos)       /*!< 0x00010000 */

#define RCC_CIR_LSERDYC_Pos                (17U)
#define RCC_CIR_LSERDYC                    (0x1UL << RCC_CIR_LSERDYC_Pos)       /*!< 0x00020000 */

#define RCC_CIR_HSIRDYC_Pos                (18U)
#define RCC_CIR_HSIRDYC                    (0x1UL << RCC_CIR_HSIRDYC_Pos)       /*!< 0x00040000 */

#define RCC_CIR_HSERDYC_Pos                (19U)
#define RCC_CIR_HSERDYC                    (0x1UL << RCC_CIR_HSERDYC_Pos)       /*!< 0x00080000 */

#define RCC_CIR_PLLRDYC_Pos                (20U)
#define RCC_CIR_PLLRDYC                    (0x1UL << RCC_CIR_PLLRDYC_Pos)       /*!< 0x00100000 */

#define RCC_CIR_PLLI2SRDYC_Pos             (21U)
#define RCC_CIR_PLLI2SRDYC                 (0x1UL << RCC_CIR_PLLI2SRDYC_Pos)    /*!< 0x00200000 */

#define RCC_CIR_PLLSAIRDYC_Pos             (22U)
#define RCC_CIR_PLLSAIRDYC                 (0x1UL << RCC_CIR_PLLSAIRDYC_Pos)    /*!< 0x00400000 */

#define RCC_CIR_CSSC_Pos                   (23U)
#define RCC_CIR_CSSC                       (0x1UL << RCC_CIR_CSSC_Pos)          /*!< 0x00800000 */

/********************  Bit definition for RCC_AHB1RSTR register  **************/
#define RCC_AHB1RSTR_GPIOARST_Pos          (0U)
#define RCC_AHB1RSTR_GPIOARST              (0x1UL << RCC_AHB1RSTR_GPIOARST_Pos) /*!< 0x00000001 */

#define RCC_AHB1RSTR_GPIOBRST_Pos          (1U)
#define RCC_AHB1RSTR_GPIOBRST              (0x1UL << RCC_AHB1RSTR_GPIOBRST_Pos) /*!< 0x00000002 */

#define RCC_AHB1RSTR_GPIOCRST_Pos          (2U)
#define RCC_AHB1RSTR_GPIOCRST              (0x1UL << RCC_AHB1RSTR_GPIOCRST_Pos) /*!< 0x00000004 */

#define RCC_AHB1RSTR_GPIODRST_Pos          (3U)
#define RCC_AHB1RSTR_GPIODRST              (0x1UL << RCC_AHB1RSTR_GPIODRST_Pos) /*!< 0x00000008 */

#define RCC_AHB1RSTR_GPIOERST_Pos          (4U)
#define RCC_AHB1RSTR_GPIOERST              (0x1UL << RCC_AHB1RSTR_GPIOERST_Pos) /*!< 0x00000010 */

#define RCC_AHB1RSTR_GPIOFRST_Pos          (5U)
#define RCC_AHB1RSTR_GPIOFRST              (0x1UL << RCC_AHB1RSTR_GPIOFRST_Pos) /*!< 0x00000020 */

#define RCC_AHB1RSTR_GPIOGRST_Pos          (6U)
#define RCC_AHB1RSTR_GPIOGRST              (0x1UL << RCC_AHB1RSTR_GPIOGRST_Pos) /*!< 0x00000040 */

#define RCC_AHB1RSTR_GPIOHRST_Pos          (7U)
#define RCC_AHB1RSTR_GPIOHRST              (0x1UL << RCC_AHB1RSTR_GPIOHRST_Pos) /*!< 0x00000080 */

#define RCC_AHB1RSTR_GPIOIRST_Pos          (8U)
#define RCC_AHB1RSTR_GPIOIRST              (0x1UL << RCC_AHB1RSTR_GPIOIRST_Pos) /*!< 0x00000100 */

#define RCC_AHB1RSTR_GPIOJRST_Pos          (9U)
#define RCC_AHB1RSTR_GPIOJRST              (0x1UL << RCC_AHB1RSTR_GPIOJRST_Pos) /*!< 0x00000200 */

#define RCC_AHB1RSTR_GPIOKRST_Pos          (10U)
#define RCC_AHB1RSTR_GPIOKRST              (0x1UL << RCC_AHB1RSTR_GPIOKRST_Pos) /*!< 0x00000400 */

#define RCC_AHB1RSTR_CRCRST_Pos            (12U)
#define RCC_AHB1RSTR_CRCRST                (0x1UL << RCC_AHB1RSTR_CRCRST_Pos)   /*!< 0x00001000 */

#define RCC_AHB1RSTR_DMA1RST_Pos           (21U)
#define RCC_AHB1RSTR_DMA1RST               (0x1UL << RCC_AHB1RSTR_DMA1RST_Pos)  /*!< 0x00200000 */

#define RCC_AHB1RSTR_DMA2RST_Pos           (22U)
#define RCC_AHB1RSTR_DMA2RST               (0x1UL << RCC_AHB1RSTR_DMA2RST_Pos)  /*!< 0x00400000 */

#define RCC_AHB1RSTR_DMA2DRST_Pos          (23U)
#define RCC_AHB1RSTR_DMA2DRST              (0x1UL << RCC_AHB1RSTR_DMA2DRST_Pos) /*!< 0x00800000 */

#define RCC_AHB1RSTR_ETHMACRST_Pos         (25U)
#define RCC_AHB1RSTR_ETHMACRST             (0x1UL << RCC_AHB1RSTR_ETHMACRST_Pos) /*!< 0x02000000 */

#define RCC_AHB1RSTR_OTGHRST_Pos           (29U)
#define RCC_AHB1RSTR_OTGHRST               (0x1UL << RCC_AHB1RSTR_OTGHRST_Pos)  /*!< 0x20000000 */

/********************  Bit definition for RCC_AHB2RSTR register  **************/
#define RCC_AHB2RSTR_DCMIRST_Pos           (0U)
#define RCC_AHB2RSTR_DCMIRST               (0x1UL << RCC_AHB2RSTR_DCMIRST_Pos)  /*!< 0x00000001 */

#define RCC_AHB2RSTR_RNGRST_Pos            (6U)
#define RCC_AHB2RSTR_RNGRST                (0x1UL << RCC_AHB2RSTR_RNGRST_Pos)   /*!< 0x00000040 */

#define RCC_AHB2RSTR_OTGFSRST_Pos          (7U)
#define RCC_AHB2RSTR_OTGFSRST              (0x1UL << RCC_AHB2RSTR_OTGFSRST_Pos) /*!< 0x00000080 */

/********************  Bit definition for RCC_AHB3RSTR register  **************/
#define RCC_AHB3RSTR_FMCRST_Pos            (0U)
#define RCC_AHB3RSTR_FMCRST                (0x1UL << RCC_AHB3RSTR_FMCRST_Pos)   /*!< 0x00000001 */

/********************  Bit definition for RCC_APB1RSTR register  **************/
#define RCC_APB1RSTR_TIM2RST_Pos           (0U)
#define RCC_APB1RSTR_TIM2RST               (0x1UL << RCC_APB1RSTR_TIM2RST_Pos)  /*!< 0x00000001 */

#define RCC_APB1RSTR_TIM3RST_Pos           (1U)
#define RCC_APB1RSTR_TIM3RST               (0x1UL << RCC_APB1RSTR_TIM3RST_Pos)  /*!< 0x00000002 */

#define RCC_APB1RSTR_TIM4RST_Pos           (2U)
#define RCC_APB1RSTR_TIM4RST               (0x1UL << RCC_APB1RSTR_TIM4RST_Pos)  /*!< 0x00000004 */

#define RCC_APB1RSTR_TIM5RST_Pos           (3U)
#define RCC_APB1RSTR_TIM5RST               (0x1UL << RCC_APB1RSTR_TIM5RST_Pos)  /*!< 0x00000008 */

#define RCC_APB1RSTR_TIM6RST_Pos           (4U)
#define RCC_APB1RSTR_TIM6RST               (0x1UL << RCC_APB1RSTR_TIM6RST_Pos)  /*!< 0x00000010 */

#define RCC_APB1RSTR_TIM7RST_Pos           (5U)
#define RCC_APB1RSTR_TIM7RST               (0x1UL << RCC_APB1RSTR_TIM7RST_Pos)  /*!< 0x00000020 */

#define RCC_APB1RSTR_TIM12RST_Pos          (6U)
#define RCC_APB1RSTR_TIM12RST              (0x1UL << RCC_APB1RSTR_TIM12RST_Pos) /*!< 0x00000040 */

#define RCC_APB1RSTR_TIM13RST_Pos          (7U)
#define RCC_APB1RSTR_TIM13RST              (0x1UL << RCC_APB1RSTR_TIM13RST_Pos) /*!< 0x00000080 */

#define RCC_APB1RSTR_TIM14RST_Pos          (8U)
#define RCC_APB1RSTR_TIM14RST              (0x1UL << RCC_APB1RSTR_TIM14RST_Pos) /*!< 0x00000100 */

#define RCC_APB1RSTR_WWDGRST_Pos           (11U)
#define RCC_APB1RSTR_WWDGRST               (0x1UL << RCC_APB1RSTR_WWDGRST_Pos)  /*!< 0x00000800 */

#define RCC_APB1RSTR_SPI2RST_Pos           (14U)
#define RCC_APB1RSTR_SPI2RST               (0x1UL << RCC_APB1RSTR_SPI2RST_Pos)  /*!< 0x00004000 */

#define RCC_APB1RSTR_SPI3RST_Pos           (15U)
#define RCC_APB1RSTR_SPI3RST               (0x1UL << RCC_APB1RSTR_SPI3RST_Pos)  /*!< 0x00008000 */

#define RCC_APB1RSTR_USART2RST_Pos         (17U)
#define RCC_APB1RSTR_USART2RST             (0x1UL << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */

#define RCC_APB1RSTR_USART3RST_Pos         (18U)
#define RCC_APB1RSTR_USART3RST             (0x1UL << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */

#define RCC_APB1RSTR_UART4RST_Pos          (19U)
#define RCC_APB1RSTR_UART4RST              (0x1UL << RCC_APB1RSTR_UART4RST_Pos) /*!< 0x00080000 */

#define RCC_APB1RSTR_UART5RST_Pos          (20U)
#define RCC_APB1RSTR_UART5RST              (0x1UL << RCC_APB1RSTR_UART5RST_Pos) /*!< 0x00100000 */

#define RCC_APB1RSTR_I2C1RST_Pos           (21U)
#define RCC_APB1RSTR_I2C1RST               (0x1UL << RCC_APB1RSTR_I2C1RST_Pos)  /*!< 0x00200000 */

#define RCC_APB1RSTR_I2C2RST_Pos           (22U)
#define RCC_APB1RSTR_I2C2RST               (0x1UL << RCC_APB1RSTR_I2C2RST_Pos)  /*!< 0x00400000 */

#define RCC_APB1RSTR_I2C3RST_Pos           (23U)
#define RCC_APB1RSTR_I2C3RST               (0x1UL << RCC_APB1RSTR_I2C3RST_Pos)  /*!< 0x00800000 */

#define RCC_APB1RSTR_CAN1RST_Pos           (25U)
#define RCC_APB1RSTR_CAN1RST               (0x1UL << RCC_APB1RSTR_CAN1RST_Pos)  /*!< 0x02000000 */

#define RCC_APB1RSTR_CAN2RST_Pos           (26U)
#define RCC_APB1RSTR_CAN2RST               (0x1UL << RCC_APB1RSTR_CAN2RST_Pos)  /*!< 0x04000000 */

#define RCC_APB1RSTR_PWRRST_Pos            (28U)
#define RCC_APB1RSTR_PWRRST                (0x1UL << RCC_APB1RSTR_PWRRST_Pos)   /*!< 0x10000000 */

#define RCC_APB1RSTR_DACRST_Pos            (29U)
#define RCC_APB1RSTR_DACRST                (0x1UL << RCC_APB1RSTR_DACRST_Pos)   /*!< 0x20000000 */

#define RCC_APB1RSTR_UART7RST_Pos          (30U)
#define RCC_APB1RSTR_UART7RST              (0x1UL << RCC_APB1RSTR_UART7RST_Pos) /*!< 0x40000000 */

#define RCC_APB1RSTR_UART8RST_Pos          (31U)
#define RCC_APB1RSTR_UART8RST              (0x1UL << RCC_APB1RSTR_UART8RST_Pos) /*!< 0x80000000 */

/********************  Bit definition for RCC_APB2RSTR register  **************/
#define RCC_APB2RSTR_TIM1RST_Pos           (0U)
#define RCC_APB2RSTR_TIM1RST               (0x1UL << RCC_APB2RSTR_TIM1RST_Pos)  /*!< 0x00000001 */

#define RCC_APB2RSTR_TIM8RST_Pos           (1U)
#define RCC_APB2RSTR_TIM8RST               (0x1UL << RCC_APB2RSTR_TIM8RST_Pos)  /*!< 0x00000002 */

#define RCC_APB2RSTR_USART1RST_Pos         (4U)
#define RCC_APB2RSTR_USART1RST             (0x1UL << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00000010 */

#define RCC_APB2RSTR_USART6RST_Pos         (5U)
#define RCC_APB2RSTR_USART6RST             (0x1UL << RCC_APB2RSTR_USART6RST_Pos) /*!< 0x00000020 */

#define RCC_APB2RSTR_ADCRST_Pos            (8U)
#define RCC_APB2RSTR_ADCRST                (0x1UL << RCC_APB2RSTR_ADCRST_Pos)   /*!< 0x00000100 */

#define RCC_APB2RSTR_SDIORST_Pos           (11U)
#define RCC_APB2RSTR_SDIORST               (0x1UL << RCC_APB2RSTR_SDIORST_Pos)  /*!< 0x00000800 */

#define RCC_APB2RSTR_SPI1RST_Pos           (12U)
#define RCC_APB2RSTR_SPI1RST               (0x1UL << RCC_APB2RSTR_SPI1RST_Pos)  /*!< 0x00001000 */

#define RCC_APB2RSTR_SPI4RST_Pos           (13U)
#define RCC_APB2RSTR_SPI4RST               (0x1UL << RCC_APB2RSTR_SPI4RST_Pos)  /*!< 0x00002000 */

#define RCC_APB2RSTR_SYSCFGRST_Pos         (14U)
#define RCC_APB2RSTR_SYSCFGRST             (0x1UL << RCC_APB2RSTR_SYSCFGRST_Pos) /*!< 0x00004000 */

#define RCC_APB2RSTR_TIM9RST_Pos           (16U)
#define RCC_APB2RSTR_TIM9RST               (0x1UL << RCC_APB2RSTR_TIM9RST_Pos)  /*!< 0x00010000 */

#define RCC_APB2RSTR_TIM10RST_Pos          (17U)
#define RCC_APB2RSTR_TIM10RST              (0x1UL << RCC_APB2RSTR_TIM10RST_Pos) /*!< 0x00020000 */

#define RCC_APB2RSTR_TIM11RST_Pos          (18U)
#define RCC_APB2RSTR_TIM11RST              (0x1UL << RCC_APB2RSTR_TIM11RST_Pos) /*!< 0x00040000 */

#define RCC_APB2RSTR_SPI5RST_Pos           (20U)
#define RCC_APB2RSTR_SPI5RST               (0x1UL << RCC_APB2RSTR_SPI5RST_Pos)  /*!< 0x00100000 */

#define RCC_APB2RSTR_SPI6RST_Pos           (21U)
#define RCC_APB2RSTR_SPI6RST               (0x1UL << RCC_APB2RSTR_SPI6RST_Pos)  /*!< 0x00200000 */

#define RCC_APB2RSTR_SAI1RST_Pos           (22U)
#define RCC_APB2RSTR_SAI1RST               (0x1UL << RCC_APB2RSTR_SAI1RST_Pos)  /*!< 0x00400000 */

#define RCC_APB2RSTR_LTDCRST_Pos           (26U)
#define RCC_APB2RSTR_LTDCRST               (0x1UL << RCC_APB2RSTR_LTDCRST_Pos)  /*!< 0x04000000 */

/* Old SPI1RST bit definition, maintained for legacy purpose */
#define  RCC_APB2RSTR_SPI1                 RCC_APB2RSTR_SPI1RST

/********************  Bit definition for RCC_AHB1ENR register  ***************/
#define RCC_AHB1ENR_GPIOAEN_Pos            (0U)
#define RCC_AHB1ENR_GPIOAEN                (0x1UL << RCC_AHB1ENR_GPIOAEN_Pos)   /*!< 0x00000001 */

#define RCC_AHB1ENR_GPIOBEN_Pos            (1U)
#define RCC_AHB1ENR_GPIOBEN                (0x1UL << RCC_AHB1ENR_GPIOBEN_Pos)   /*!< 0x00000002 */

#define RCC_AHB1ENR_GPIOCEN_Pos            (2U)
#define RCC_AHB1ENR_GPIOCEN                (0x1UL << RCC_AHB1ENR_GPIOCEN_Pos)   /*!< 0x00000004 */

#define RCC_AHB1ENR_GPIODEN_Pos            (3U)
#define RCC_AHB1ENR_GPIODEN                (0x1UL << RCC_AHB1ENR_GPIODEN_Pos)   /*!< 0x00000008 */

#define RCC_AHB1ENR_GPIOEEN_Pos            (4U)
#define RCC_AHB1ENR_GPIOEEN                (0x1UL << RCC_AHB1ENR_GPIOEEN_Pos)   /*!< 0x00000010 */

#define RCC_AHB1ENR_GPIOFEN_Pos            (5U)
#define RCC_AHB1ENR_GPIOFEN                (0x1UL << RCC_AHB1ENR_GPIOFEN_Pos)   /*!< 0x00000020 */

#define RCC_AHB1ENR_GPIOGEN_Pos            (6U)
#define RCC_AHB1ENR_GPIOGEN                (0x1UL << RCC_AHB1ENR_GPIOGEN_Pos)   /*!< 0x00000040 */

#define RCC_AHB1ENR_GPIOHEN_Pos            (7U)
#define RCC_AHB1ENR_GPIOHEN                (0x1UL << RCC_AHB1ENR_GPIOHEN_Pos)   /*!< 0x00000080 */

#define RCC_AHB1ENR_GPIOIEN_Pos            (8U)
#define RCC_AHB1ENR_GPIOIEN                (0x1UL << RCC_AHB1ENR_GPIOIEN_Pos)   /*!< 0x00000100 */

#define RCC_AHB1ENR_GPIOJEN_Pos            (9U)
#define RCC_AHB1ENR_GPIOJEN                (0x1UL << RCC_AHB1ENR_GPIOJEN_Pos)   /*!< 0x00000200 */

#define RCC_AHB1ENR_GPIOKEN_Pos            (10U)
#define RCC_AHB1ENR_GPIOKEN                (0x1UL << RCC_AHB1ENR_GPIOKEN_Pos)   /*!< 0x00000400 */

#define RCC_AHB1ENR_CRCEN_Pos              (12U)
#define RCC_AHB1ENR_CRCEN                  (0x1UL << RCC_AHB1ENR_CRCEN_Pos)     /*!< 0x00001000 */

#define RCC_AHB1ENR_BKPSRAMEN_Pos          (18U)
#define RCC_AHB1ENR_BKPSRAMEN              (0x1UL << RCC_AHB1ENR_BKPSRAMEN_Pos) /*!< 0x00040000 */

#define RCC_AHB1ENR_CCMDATARAMEN_Pos       (20U)
#define RCC_AHB1ENR_CCMDATARAMEN           (0x1UL << RCC_AHB1ENR_CCMDATARAMEN_Pos) /*!< 0x00100000 */

#define RCC_AHB1ENR_DMA1EN_Pos             (21U)
#define RCC_AHB1ENR_DMA1EN                 (0x1UL << RCC_AHB1ENR_DMA1EN_Pos)    /*!< 0x00200000 */

#define RCC_AHB1ENR_DMA2EN_Pos             (22U)
#define RCC_AHB1ENR_DMA2EN                 (0x1UL << RCC_AHB1ENR_DMA2EN_Pos)    /*!< 0x00400000 */

#define RCC_AHB1ENR_DMA2DEN_Pos            (23U)
#define RCC_AHB1ENR_DMA2DEN                (0x1UL << RCC_AHB1ENR_DMA2DEN_Pos)   /*!< 0x00800000 */

#define RCC_AHB1ENR_ETHMACEN_Pos           (25U)
#define RCC_AHB1ENR_ETHMACEN               (0x1UL << RCC_AHB1ENR_ETHMACEN_Pos)  /*!< 0x02000000 */

#define RCC_AHB1ENR_ETHMACTXEN_Pos         (26U)
#define RCC_AHB1ENR_ETHMACTXEN             (0x1UL << RCC_AHB1ENR_ETHMACTXEN_Pos) /*!< 0x04000000 */

#define RCC_AHB1ENR_ETHMACRXEN_Pos         (27U)
#define RCC_AHB1ENR_ETHMACRXEN             (0x1UL << RCC_AHB1ENR_ETHMACRXEN_Pos) /*!< 0x08000000 */

#define RCC_AHB1ENR_ETHMACPTPEN_Pos        (28U)
#define RCC_AHB1ENR_ETHMACPTPEN            (0x1UL << RCC_AHB1ENR_ETHMACPTPEN_Pos) /*!< 0x10000000 */

#define RCC_AHB1ENR_OTGHSEN_Pos            (29U)
#define RCC_AHB1ENR_OTGHSEN                (0x1UL << RCC_AHB1ENR_OTGHSEN_Pos)   /*!< 0x20000000 */

#define RCC_AHB1ENR_OTGHSULPIEN_Pos        (30U)
#define RCC_AHB1ENR_OTGHSULPIEN            (0x1UL << RCC_AHB1ENR_OTGHSULPIEN_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_AHB2ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_AHB2_SUPPORT                   /*!< AHB2 Bus is supported */

#define RCC_AHB2ENR_DCMIEN_Pos             (0U)
#define RCC_AHB2ENR_DCMIEN                 (0x1UL << RCC_AHB2ENR_DCMIEN_Pos)    /*!< 0x00000001 */

#define RCC_AHB2ENR_RNGEN_Pos              (6U)
#define RCC_AHB2ENR_RNGEN                  (0x1UL << RCC_AHB2ENR_RNGEN_Pos)     /*!< 0x00000040 */

#define RCC_AHB2ENR_OTGFSEN_Pos            (7U)
#define RCC_AHB2ENR_OTGFSEN                (0x1UL << RCC_AHB2ENR_OTGFSEN_Pos)   /*!< 0x00000080 */

/********************  Bit definition for RCC_AHB3ENR register  ***************/
/*
 * @brief Specific device feature definitions (not present on all devices in the STM32F4 serie)
 */
#define RCC_AHB3_SUPPORT                   /*!< AHB3 Bus is supported */

#define RCC_AHB3ENR_FMCEN_Pos              (0U)
#define RCC_AHB3ENR_FMCEN                  (0x1UL << RCC_AHB3ENR_FMCEN_Pos)     /*!< 0x00000001 */


/********************  Bit definition for RCC_APB1ENR register  ***************/
#define RCC_APB1ENR_TIM2EN_Pos             (0U)
#define RCC_APB1ENR_TIM2EN                 (0x1UL << RCC_APB1ENR_TIM2EN_Pos)    /*!< 0x00000001 */

#define RCC_APB1ENR_TIM3EN_Pos             (1U)
#define RCC_APB1ENR_TIM3EN                 (0x1UL << RCC_APB1ENR_TIM3EN_Pos)    /*!< 0x00000002 */

#define RCC_APB1ENR_TIM4EN_Pos             (2U)
#define RCC_APB1ENR_TIM4EN                 (0x1UL << RCC_APB1ENR_TIM4EN_Pos)    /*!< 0x00000004 */

#define RCC_APB1ENR_TIM5EN_Pos             (3U)
#define RCC_APB1ENR_TIM5EN                 (0x1UL << RCC_APB1ENR_TIM5EN_Pos)    /*!< 0x00000008 */

#define RCC_APB1ENR_TIM6EN_Pos             (4U)
#define RCC_APB1ENR_TIM6EN                 (0x1UL << RCC_APB1ENR_TIM6EN_Pos)    /*!< 0x00000010 */

#define RCC_APB1ENR_TIM7EN_Pos             (5U)
#define RCC_APB1ENR_TIM7EN                 (0x1UL << RCC_APB1ENR_TIM7EN_Pos)    /*!< 0x00000020 */

#define RCC_APB1ENR_TIM12EN_Pos            (6U)
#define RCC_APB1ENR_TIM12EN                (0x1UL << RCC_APB1ENR_TIM12EN_Pos)   /*!< 0x00000040 */

#define RCC_APB1ENR_TIM13EN_Pos            (7U)
#define RCC_APB1ENR_TIM13EN                (0x1UL << RCC_APB1ENR_TIM13EN_Pos)   /*!< 0x00000080 */

#define RCC_APB1ENR_TIM14EN_Pos            (8U)
#define RCC_APB1ENR_TIM14EN                (0x1UL << RCC_APB1ENR_TIM14EN_Pos)   /*!< 0x00000100 */

#define RCC_APB1ENR_WWDGEN_Pos             (11U)
#define RCC_APB1ENR_WWDGEN                 (0x1UL << RCC_APB1ENR_WWDGEN_Pos)    /*!< 0x00000800 */

#define RCC_APB1ENR_SPI2EN_Pos             (14U)
#define RCC_APB1ENR_SPI2EN                 (0x1UL << RCC_APB1ENR_SPI2EN_Pos)    /*!< 0x00004000 */

#define RCC_APB1ENR_SPI3EN_Pos             (15U)
#define RCC_APB1ENR_SPI3EN                 (0x1UL << RCC_APB1ENR_SPI3EN_Pos)    /*!< 0x00008000 */

#define RCC_APB1ENR_USART2EN_Pos           (17U)
#define RCC_APB1ENR_USART2EN               (0x1UL << RCC_APB1ENR_USART2EN_Pos)  /*!< 0x00020000 */

#define RCC_APB1ENR_USART3EN_Pos           (18U)
#define RCC_APB1ENR_USART3EN               (0x1UL << RCC_APB1ENR_USART3EN_Pos)  /*!< 0x00040000 */

#define RCC_APB1ENR_UART4EN_Pos            (19U)
#define RCC_APB1ENR_UART4EN                (0x1UL << RCC_APB1ENR_UART4EN_Pos)   /*!< 0x00080000 */

#define RCC_APB1ENR_UART5EN_Pos            (20U)
#define RCC_APB1ENR_UART5EN                (0x1UL << RCC_APB1ENR_UART5EN_Pos)   /*!< 0x00100000 */

#define RCC_APB1ENR_I2C1EN_Pos             (21U)
#define RCC_APB1ENR_I2C1EN                 (0x1UL << RCC_APB1ENR_I2C1EN_Pos)    /*!< 0x00200000 */

#define RCC_APB1ENR_I2C2EN_Pos             (22U)
#define RCC_APB1ENR_I2C2EN                 (0x1UL << RCC_APB1ENR_I2C2EN_Pos)    /*!< 0x00400000 */

#define RCC_APB1ENR_I2C3EN_Pos             (23U)
#define RCC_APB1ENR_I2C3EN                 (0x1UL << RCC_APB1ENR_I2C3EN_Pos)    /*!< 0x00800000 */

#define RCC_APB1ENR_CAN1EN_Pos             (25U)
#define RCC_APB1ENR_CAN1EN                 (0x1UL << RCC_APB1ENR_CAN1EN_Pos)    /*!< 0x02000000 */

#define RCC_APB1ENR_CAN2EN_Pos             (26U)
#define RCC_APB1ENR_CAN2EN                 (0x1UL << RCC_APB1ENR_CAN2EN_Pos)    /*!< 0x04000000 */

#define RCC_APB1ENR_PWREN_Pos              (28U)
#define RCC_APB1ENR_PWREN                  (0x1UL << RCC_APB1ENR_PWREN_Pos)     /*!< 0x10000000 */

#define RCC_APB1ENR_DACEN_Pos              (29U)
#define RCC_APB1ENR_DACEN                  (0x1UL << RCC_APB1ENR_DACEN_Pos)     /*!< 0x20000000 */

#define RCC_APB1ENR_UART7EN_Pos            (30U)
#define RCC_APB1ENR_UART7EN                (0x1UL << RCC_APB1ENR_UART7EN_Pos)   /*!< 0x40000000 */

#define RCC_APB1ENR_UART8EN_Pos            (31U)
#define RCC_APB1ENR_UART8EN                (0x1UL << RCC_APB1ENR_UART8EN_Pos)   /*!< 0x80000000 */

/********************  Bit definition for RCC_APB2ENR register  ***************/
#define RCC_APB2ENR_TIM1EN_Pos             (0U)
#define RCC_APB2ENR_TIM1EN                 (0x1UL << RCC_APB2ENR_TIM1EN_Pos)    /*!< 0x00000001 */

#define RCC_APB2ENR_TIM8EN_Pos             (1U)
#define RCC_APB2ENR_TIM8EN                 (0x1UL << RCC_APB2ENR_TIM8EN_Pos)    /*!< 0x00000002 */

#define RCC_APB2ENR_USART1EN_Pos           (4U)
#define RCC_APB2ENR_USART1EN               (0x1UL << RCC_APB2ENR_USART1EN_Pos)  /*!< 0x00000010 */

#define RCC_APB2ENR_USART6EN_Pos           (5U)
#define RCC_APB2ENR_USART6EN               (0x1UL << RCC_APB2ENR_USART6EN_Pos)  /*!< 0x00000020 */

#define RCC_APB2ENR_ADC1EN_Pos             (8U)
#define RCC_APB2ENR_ADC1EN                 (0x1UL << RCC_APB2ENR_ADC1EN_Pos)    /*!< 0x00000100 */

#define RCC_APB2ENR_ADC2EN_Pos             (9U)
#define RCC_APB2ENR_ADC2EN                 (0x1UL << RCC_APB2ENR_ADC2EN_Pos)    /*!< 0x00000200 */

#define RCC_APB2ENR_ADC3EN_Pos             (10U)
#define RCC_APB2ENR_ADC3EN                 (0x1UL << RCC_APB2ENR_ADC3EN_Pos)    /*!< 0x00000400 */

#define RCC_APB2ENR_SDIOEN_Pos             (11U)
#define RCC_APB2ENR_SDIOEN                 (0x1UL << RCC_APB2ENR_SDIOEN_Pos)    /*!< 0x00000800 */

#define RCC_APB2ENR_SPI1EN_Pos             (12U)
#define RCC_APB2ENR_SPI1EN                 (0x1UL << RCC_APB2ENR_SPI1EN_Pos)    /*!< 0x00001000 */

#define RCC_APB2ENR_SPI4EN_Pos             (13U)
#define RCC_APB2ENR_SPI4EN                 (0x1UL << RCC_APB2ENR_SPI4EN_Pos)    /*!< 0x00002000 */

#define RCC_APB2ENR_SYSCFGEN_Pos           (14U)
#define RCC_APB2ENR_SYSCFGEN               (0x1UL << RCC_APB2ENR_SYSCFGEN_Pos)  /*!< 0x00004000 */

#define RCC_APB2ENR_TIM9EN_Pos             (16U)
#define RCC_APB2ENR_TIM9EN                 (0x1UL << RCC_APB2ENR_TIM9EN_Pos)    /*!< 0x00010000 */

#define RCC_APB2ENR_TIM10EN_Pos            (17U)
#define RCC_APB2ENR_TIM10EN                (0x1UL << RCC_APB2ENR_TIM10EN_Pos)   /*!< 0x00020000 */

#define RCC_APB2ENR_TIM11EN_Pos            (18U)
#define RCC_APB2ENR_TIM11EN                (0x1UL << RCC_APB2ENR_TIM11EN_Pos)   /*!< 0x00040000 */

#define RCC_APB2ENR_SPI5EN_Pos             (20U)
#define RCC_APB2ENR_SPI5EN                 (0x1UL << RCC_APB2ENR_SPI5EN_Pos)    /*!< 0x00100000 */

#define RCC_APB2ENR_SPI6EN_Pos             (21U)
#define RCC_APB2ENR_SPI6EN                 (0x1UL << RCC_APB2ENR_SPI6EN_Pos)    /*!< 0x00200000 */

#define RCC_APB2ENR_SAI1EN_Pos             (22U)
#define RCC_APB2ENR_SAI1EN                 (0x1UL << RCC_APB2ENR_SAI1EN_Pos)    /*!< 0x00400000 */

#define RCC_APB2ENR_LTDCEN_Pos             (26U)
#define RCC_APB2ENR_LTDCEN                 (0x1UL << RCC_APB2ENR_LTDCEN_Pos)    /*!< 0x04000000 */

/********************  Bit definition for RCC_AHB1LPENR register  *************/
#define RCC_AHB1LPENR_GPIOALPEN_Pos        (0U)
#define RCC_AHB1LPENR_GPIOALPEN            (0x1UL << RCC_AHB1LPENR_GPIOALPEN_Pos) /*!< 0x00000001 */

#define RCC_AHB1LPENR_GPIOBLPEN_Pos        (1U)
#define RCC_AHB1LPENR_GPIOBLPEN            (0x1UL << RCC_AHB1LPENR_GPIOBLPEN_Pos) /*!< 0x00000002 */

#define RCC_AHB1LPENR_GPIOCLPEN_Pos        (2U)
#define RCC_AHB1LPENR_GPIOCLPEN            (0x1UL << RCC_AHB1LPENR_GPIOCLPEN_Pos) /*!< 0x00000004 */

#define RCC_AHB1LPENR_GPIODLPEN_Pos        (3U)
#define RCC_AHB1LPENR_GPIODLPEN            (0x1UL << RCC_AHB1LPENR_GPIODLPEN_Pos) /*!< 0x00000008 */

#define RCC_AHB1LPENR_GPIOELPEN_Pos        (4U)
#define RCC_AHB1LPENR_GPIOELPEN            (0x1UL << RCC_AHB1LPENR_GPIOELPEN_Pos) /*!< 0x00000010 */

#define RCC_AHB1LPENR_GPIOFLPEN_Pos        (5U)
#define RCC_AHB1LPENR_GPIOFLPEN            (0x1UL << RCC_AHB1LPENR_GPIOFLPEN_Pos) /*!< 0x00000020 */

#define RCC_AHB1LPENR_GPIOGLPEN_Pos        (6U)
#define RCC_AHB1LPENR_GPIOGLPEN            (0x1UL << RCC_AHB1LPENR_GPIOGLPEN_Pos) /*!< 0x00000040 */

#define RCC_AHB1LPENR_GPIOHLPEN_Pos        (7U)
#define RCC_AHB1LPENR_GPIOHLPEN            (0x1UL << RCC_AHB1LPENR_GPIOHLPEN_Pos) /*!< 0x00000080 */

#define RCC_AHB1LPENR_GPIOILPEN_Pos        (8U)
#define RCC_AHB1LPENR_GPIOILPEN            (0x1UL << RCC_AHB1LPENR_GPIOILPEN_Pos) /*!< 0x00000100 */

#define RCC_AHB1LPENR_GPIOJLPEN_Pos        (9U)
#define RCC_AHB1LPENR_GPIOJLPEN            (0x1UL << RCC_AHB1LPENR_GPIOJLPEN_Pos) /*!< 0x00000200 */

#define RCC_AHB1LPENR_GPIOKLPEN_Pos        (10U)
#define RCC_AHB1LPENR_GPIOKLPEN            (0x1UL << RCC_AHB1LPENR_GPIOKLPEN_Pos) /*!< 0x00000400 */

#define RCC_AHB1LPENR_CRCLPEN_Pos          (12U)
#define RCC_AHB1LPENR_CRCLPEN              (0x1UL << RCC_AHB1LPENR_CRCLPEN_Pos) /*!< 0x00001000 */

#define RCC_AHB1LPENR_FLITFLPEN_Pos        (15U)
#define RCC_AHB1LPENR_FLITFLPEN            (0x1UL << RCC_AHB1LPENR_FLITFLPEN_Pos) /*!< 0x00008000 */

#define RCC_AHB1LPENR_SRAM1LPEN_Pos        (16U)
#define RCC_AHB1LPENR_SRAM1LPEN            (0x1UL << RCC_AHB1LPENR_SRAM1LPEN_Pos) /*!< 0x00010000 */

#define RCC_AHB1LPENR_SRAM2LPEN_Pos        (17U)
#define RCC_AHB1LPENR_SRAM2LPEN            (0x1UL << RCC_AHB1LPENR_SRAM2LPEN_Pos) /*!< 0x00020000 */

#define RCC_AHB1LPENR_BKPSRAMLPEN_Pos      (18U)
#define RCC_AHB1LPENR_BKPSRAMLPEN          (0x1UL << RCC_AHB1LPENR_BKPSRAMLPEN_Pos) /*!< 0x00040000 */

#define RCC_AHB1LPENR_SRAM3LPEN_Pos        (19U)
#define RCC_AHB1LPENR_SRAM3LPEN            (0x1UL << RCC_AHB1LPENR_SRAM3LPEN_Pos) /*!< 0x00080000 */

#define RCC_AHB1LPENR_DMA1LPEN_Pos         (21U)
#define RCC_AHB1LPENR_DMA1LPEN             (0x1UL << RCC_AHB1LPENR_DMA1LPEN_Pos) /*!< 0x00200000 */

#define RCC_AHB1LPENR_DMA2LPEN_Pos         (22U)
#define RCC_AHB1LPENR_DMA2LPEN             (0x1UL << RCC_AHB1LPENR_DMA2LPEN_Pos) /*!< 0x00400000 */

#define RCC_AHB1LPENR_DMA2DLPEN_Pos        (23U)
#define RCC_AHB1LPENR_DMA2DLPEN            (0x1UL << RCC_AHB1LPENR_DMA2DLPEN_Pos) /*!< 0x00800000 */

#define RCC_AHB1LPENR_ETHMACLPEN_Pos       (25U)
#define RCC_AHB1LPENR_ETHMACLPEN           (0x1UL << RCC_AHB1LPENR_ETHMACLPEN_Pos) /*!< 0x02000000 */

#define RCC_AHB1LPENR_ETHMACTXLPEN_Pos     (26U)
#define RCC_AHB1LPENR_ETHMACTXLPEN         (0x1UL << RCC_AHB1LPENR_ETHMACTXLPEN_Pos) /*!< 0x04000000 */

#define RCC_AHB1LPENR_ETHMACRXLPEN_Pos     (27U)
#define RCC_AHB1LPENR_ETHMACRXLPEN         (0x1UL << RCC_AHB1LPENR_ETHMACRXLPEN_Pos) /*!< 0x08000000 */

#define RCC_AHB1LPENR_ETHMACPTPLPEN_Pos    (28U)
#define RCC_AHB1LPENR_ETHMACPTPLPEN        (0x1UL << RCC_AHB1LPENR_ETHMACPTPLPEN_Pos) /*!< 0x10000000 */

#define RCC_AHB1LPENR_OTGHSLPEN_Pos        (29U)
#define RCC_AHB1LPENR_OTGHSLPEN            (0x1UL << RCC_AHB1LPENR_OTGHSLPEN_Pos) /*!< 0x20000000 */

#define RCC_AHB1LPENR_OTGHSULPILPEN_Pos    (30U)
#define RCC_AHB1LPENR_OTGHSULPILPEN        (0x1UL << RCC_AHB1LPENR_OTGHSULPILPEN_Pos) /*!< 0x40000000 */

/********************  Bit definition for RCC_AHB2LPENR register  *************/
#define RCC_AHB2LPENR_DCMILPEN_Pos         (0U)
#define RCC_AHB2LPENR_DCMILPEN             (0x1UL << RCC_AHB2LPENR_DCMILPEN_Pos) /*!< 0x00000001 */

#define RCC_AHB2LPENR_RNGLPEN_Pos          (6U)
#define RCC_AHB2LPENR_RNGLPEN              (0x1UL << RCC_AHB2LPENR_RNGLPEN_Pos) /*!< 0x00000040 */

#define RCC_AHB2LPENR_OTGFSLPEN_Pos        (7U)
#define RCC_AHB2LPENR_OTGFSLPEN            (0x1UL << RCC_AHB2LPENR_OTGFSLPEN_Pos) /*!< 0x00000080 */

/********************  Bit definition for RCC_AHB3LPENR register  *************/
#define RCC_AHB3LPENR_FMCLPEN_Pos          (0U)
#define RCC_AHB3LPENR_FMCLPEN              (0x1UL << RCC_AHB3LPENR_FMCLPEN_Pos) /*!< 0x00000001 */

/********************  Bit definition for RCC_APB1LPENR register  *************/
#define RCC_APB1LPENR_TIM2LPEN_Pos         (0U)
#define RCC_APB1LPENR_TIM2LPEN             (0x1UL << RCC_APB1LPENR_TIM2LPEN_Pos) /*!< 0x00000001 */

#define RCC_APB1LPENR_TIM3LPEN_Pos         (1U)
#define RCC_APB1LPENR_TIM3LPEN             (0x1UL << RCC_APB1LPENR_TIM3LPEN_Pos) /*!< 0x00000002 */

#define RCC_APB1LPENR_TIM4LPEN_Pos         (2U)
#define RCC_APB1LPENR_TIM4LPEN             (0x1UL << RCC_APB1LPENR_TIM4LPEN_Pos) /*!< 0x00000004 */

#define RCC_APB1LPENR_TIM5LPEN_Pos         (3U)
#define RCC_APB1LPENR_TIM5LPEN             (0x1UL << RCC_APB1LPENR_TIM5LPEN_Pos) /*!< 0x00000008 */

#define RCC_APB1LPENR_TIM6LPEN_Pos         (4U)
#define RCC_APB1LPENR_TIM6LPEN             (0x1UL << RCC_APB1LPENR_TIM6LPEN_Pos) /*!< 0x00000010 */

#define RCC_APB1LPENR_TIM7LPEN_Pos         (5U)
#define RCC_APB1LPENR_TIM7LPEN             (0x1UL << RCC_APB1LPENR_TIM7LPEN_Pos) /*!< 0x00000020 */

#define RCC_APB1LPENR_TIM12LPEN_Pos        (6U)
#define RCC_APB1LPENR_TIM12LPEN            (0x1UL << RCC_APB1LPENR_TIM12LPEN_Pos) /*!< 0x00000040 */

#define RCC_APB1LPENR_TIM13LPEN_Pos        (7U)
#define RCC_APB1LPENR_TIM13LPEN            (0x1UL << RCC_APB1LPENR_TIM13LPEN_Pos) /*!< 0x00000080 */

#define RCC_APB1LPENR_TIM14LPEN_Pos        (8U)
#define RCC_APB1LPENR_TIM14LPEN            (0x1UL << RCC_APB1LPENR_TIM14LPEN_Pos) /*!< 0x00000100 */

#define RCC_APB1LPENR_WWDGLPEN_Pos         (11U)
#define RCC_APB1LPENR_WWDGLPEN             (0x1UL << RCC_APB1LPENR_WWDGLPEN_Pos) /*!< 0x00000800 */

#define RCC_APB1LPENR_SPI2LPEN_Pos         (14U)
#define RCC_APB1LPENR_SPI2LPEN             (0x1UL << RCC_APB1LPENR_SPI2LPEN_Pos) /*!< 0x00004000 */

#define RCC_APB1LPENR_SPI3LPEN_Pos         (15U)
#define RCC_APB1LPENR_SPI3LPEN             (0x1UL << RCC_APB1LPENR_SPI3LPEN_Pos) /*!< 0x00008000 */

#define RCC_APB1LPENR_USART2LPEN_Pos       (17U)
#define RCC_APB1LPENR_USART2LPEN           (0x1UL << RCC_APB1LPENR_USART2LPEN_Pos) /*!< 0x00020000 */

#define RCC_APB1LPENR_USART3LPEN_Pos       (18U)
#define RCC_APB1LPENR_USART3LPEN           (0x1UL << RCC_APB1LPENR_USART3LPEN_Pos) /*!< 0x00040000 */

#define RCC_APB1LPENR_UART4LPEN_Pos        (19U)
#define RCC_APB1LPENR_UART4LPEN            (0x1UL << RCC_APB1LPENR_UART4LPEN_Pos) /*!< 0x00080000 */

#define RCC_APB1LPENR_UART5LPEN_Pos        (20U)
#define RCC_APB1LPENR_UART5LPEN            (0x1UL << RCC_APB1LPENR_UART5LPEN_Pos) /*!< 0x00100000 */

#define RCC_APB1LPENR_I2C1LPEN_Pos         (21U)
#define RCC_APB1LPENR_I2C1LPEN             (0x1UL << RCC_APB1LPENR_I2C1LPEN_Pos) /*!< 0x00200000 */

#define RCC_APB1LPENR_I2C2LPEN_Pos         (22U)
#define RCC_APB1LPENR_I2C2LPEN             (0x1UL << RCC_APB1LPENR_I2C2LPEN_Pos) /*!< 0x00400000 */

#define RCC_APB1LPENR_I2C3LPEN_Pos         (23U)
#define RCC_APB1LPENR_I2C3LPEN             (0x1UL << RCC_APB1LPENR_I2C3LPEN_Pos) /*!< 0x00800000 */

#define RCC_APB1LPENR_CAN1LPEN_Pos         (25U)
#define RCC_APB1LPENR_CAN1LPEN             (0x1UL << RCC_APB1LPENR_CAN1LPEN_Pos) /*!< 0x02000000 */

#define RCC_APB1LPENR_CAN2LPEN_Pos         (26U)
#define RCC_APB1LPENR_CAN2LPEN             (0x1UL << RCC_APB1LPENR_CAN2LPEN_Pos) /*!< 0x04000000 */

#define RCC_APB1LPENR_PWRLPEN_Pos          (28U)
#define RCC_APB1LPENR_PWRLPEN              (0x1UL << RCC_APB1LPENR_PWRLPEN_Pos) /*!< 0x10000000 */

#define RCC_APB1LPENR_DACLPEN_Pos          (29U)
#define RCC_APB1LPENR_DACLPEN              (0x1UL << RCC_APB1LPENR_DACLPEN_Pos) /*!< 0x20000000 */

#define RCC_APB1LPENR_UART7LPEN_Pos        (30U)
#define RCC_APB1LPENR_UART7LPEN            (0x1UL << RCC_APB1LPENR_UART7LPEN_Pos) /*!< 0x40000000 */

#define RCC_APB1LPENR_UART8LPEN_Pos        (31U)
#define RCC_APB1LPENR_UART8LPEN            (0x1UL << RCC_APB1LPENR_UART8LPEN_Pos) /*!< 0x80000000 */

/********************  Bit definition for RCC_APB2LPENR register  *************/
#define RCC_APB2LPENR_TIM1LPEN_Pos         (0U)
#define RCC_APB2LPENR_TIM1LPEN             (0x1UL << RCC_APB2LPENR_TIM1LPEN_Pos) /*!< 0x00000001 */

#define RCC_APB2LPENR_TIM8LPEN_Pos         (1U)
#define RCC_APB2LPENR_TIM8LPEN             (0x1UL << RCC_APB2LPENR_TIM8LPEN_Pos) /*!< 0x00000002 */

#define RCC_APB2LPENR_USART1LPEN_Pos       (4U)
#define RCC_APB2LPENR_USART1LPEN           (0x1UL << RCC_APB2LPENR_USART1LPEN_Pos) /*!< 0x00000010 */

#define RCC_APB2LPENR_USART6LPEN_Pos       (5U)
#define RCC_APB2LPENR_USART6LPEN           (0x1UL << RCC_APB2LPENR_USART6LPEN_Pos) /*!< 0x00000020 */

#define RCC_APB2LPENR_ADC1LPEN_Pos         (8U)
#define RCC_APB2LPENR_ADC1LPEN             (0x1UL << RCC_APB2LPENR_ADC1LPEN_Pos) /*!< 0x00000100 */

#define RCC_APB2LPENR_ADC2LPEN_Pos         (9U)
#define RCC_APB2LPENR_ADC2LPEN             (0x1UL << RCC_APB2LPENR_ADC2LPEN_Pos) /*!< 0x00000200 */

#define RCC_APB2LPENR_ADC3LPEN_Pos         (10U)
#define RCC_APB2LPENR_ADC3LPEN             (0x1UL << RCC_APB2LPENR_ADC3LPEN_Pos) /*!< 0x00000400 */

#define RCC_APB2LPENR_SDIOLPEN_Pos         (11U)
#define RCC_APB2LPENR_SDIOLPEN             (0x1UL << RCC_APB2LPENR_SDIOLPEN_Pos) /*!< 0x00000800 */

#define RCC_APB2LPENR_SPI1LPEN_Pos         (12U)
#define RCC_APB2LPENR_SPI1LPEN             (0x1UL << RCC_APB2LPENR_SPI1LPEN_Pos) /*!< 0x00001000 */

#define RCC_APB2LPENR_SPI4LPEN_Pos         (13U)
#define RCC_APB2LPENR_SPI4LPEN             (0x1UL << RCC_APB2LPENR_SPI4LPEN_Pos) /*!< 0x00002000 */

#define RCC_APB2LPENR_SYSCFGLPEN_Pos       (14U)
#define RCC_APB2LPENR_SYSCFGLPEN           (0x1UL << RCC_APB2LPENR_SYSCFGLPEN_Pos) /*!< 0x00004000 */

#define RCC_APB2LPENR_TIM9LPEN_Pos         (16U)
#define RCC_APB2LPENR_TIM9LPEN             (0x1UL << RCC_APB2LPENR_TIM9LPEN_Pos) /*!< 0x00010000 */

#define RCC_APB2LPENR_TIM10LPEN_Pos        (17U)
#define RCC_APB2LPENR_TIM10LPEN            (0x1UL << RCC_APB2LPENR_TIM10LPEN_Pos) /*!< 0x00020000 */

#define RCC_APB2LPENR_TIM11LPEN_Pos        (18U)
#define RCC_APB2LPENR_TIM11LPEN            (0x1UL << RCC_APB2LPENR_TIM11LPEN_Pos) /*!< 0x00040000 */

#define RCC_APB2LPENR_SPI5LPEN_Pos         (20U)
#define RCC_APB2LPENR_SPI5LPEN             (0x1UL << RCC_APB2LPENR_SPI5LPEN_Pos) /*!< 0x00100000 */

#define RCC_APB2LPENR_SPI6LPEN_Pos         (21U)
#define RCC_APB2LPENR_SPI6LPEN             (0x1UL << RCC_APB2LPENR_SPI6LPEN_Pos) /*!< 0x00200000 */

#define RCC_APB2LPENR_SAI1LPEN_Pos         (22U)
#define RCC_APB2LPENR_SAI1LPEN             (0x1UL << RCC_APB2LPENR_SAI1LPEN_Pos) /*!< 0x00400000 */

#define RCC_APB2LPENR_LTDCLPEN_Pos         (26U)
#define RCC_APB2LPENR_LTDCLPEN             (0x1UL << RCC_APB2LPENR_LTDCLPEN_Pos) /*!< 0x04000000 */

/********************  Bit definition for RCC_BDCR register  ******************/
#define RCC_BDCR_LSEON_Pos                 (0U)
#define RCC_BDCR_LSEON                     (0x1UL << RCC_BDCR_LSEON_Pos)        /*!< 0x00000001 */

#define RCC_BDCR_LSERDY_Pos                (1U)
#define RCC_BDCR_LSERDY                    (0x1UL << RCC_BDCR_LSERDY_Pos)       /*!< 0x00000002 */

#define RCC_BDCR_LSEBYP_Pos                (2U)
#define RCC_BDCR_LSEBYP                    (0x1UL << RCC_BDCR_LSEBYP_Pos)       /*!< 0x00000004 */

#define RCC_BDCR_RTCSEL_Pos                (8U)
#define RCC_BDCR_RTCSEL                    (0x3UL << RCC_BDCR_RTCSEL_Pos)       /*!< 0x00000300 */

#define RCC_BDCR_RTCEN_Pos                 (15U)
#define RCC_BDCR_RTCEN                     (0x1UL << RCC_BDCR_RTCEN_Pos)        /*!< 0x00008000 */

#define RCC_BDCR_BDRST_Pos                 (16U)
#define RCC_BDCR_BDRST                     (0x1UL << RCC_BDCR_BDRST_Pos)        /*!< 0x00010000 */

/********************  Bit definition for RCC_CSR register  *******************/
#define RCC_CSR_LSION_Pos                  (0U)
#define RCC_CSR_LSION                      (0x1UL << RCC_CSR_LSION_Pos)         /*!< 0x00000001 */

#define RCC_CSR_LSIRDY_Pos                 (1U)
#define RCC_CSR_LSIRDY                     (0x1UL << RCC_CSR_LSIRDY_Pos)        /*!< 0x00000002 */

#define RCC_CSR_RMVF_Pos                   (24U)
#define RCC_CSR_RMVF                       (0x1UL << RCC_CSR_RMVF_Pos)          /*!< 0x01000000 */

#define RCC_CSR_BORRSTF_Pos                (25U)
#define RCC_CSR_BORRSTF                    (0x1UL << RCC_CSR_BORRSTF_Pos)       /*!< 0x02000000 */

#define RCC_CSR_PINRSTF_Pos                (26U)
#define RCC_CSR_PINRSTF                    (0x1UL << RCC_CSR_PINRSTF_Pos)       /*!< 0x04000000 */

#define RCC_CSR_PORRSTF_Pos                (27U)
#define RCC_CSR_PORRSTF                    (0x1UL << RCC_CSR_PORRSTF_Pos)       /*!< 0x08000000 */

#define RCC_CSR_SFTRSTF_Pos                (28U)
#define RCC_CSR_SFTRSTF                    (0x1UL << RCC_CSR_SFTRSTF_Pos)       /*!< 0x10000000 */

#define RCC_CSR_IWDGRSTF_Pos               (29U)
#define RCC_CSR_IWDGRSTF                   (0x1UL << RCC_CSR_IWDGRSTF_Pos)      /*!< 0x20000000 */

#define RCC_CSR_WWDGRSTF_Pos               (30U)
#define RCC_CSR_WWDGRSTF                   (0x1UL << RCC_CSR_WWDGRSTF_Pos)      /*!< 0x40000000 */

#define RCC_CSR_LPWRRSTF_Pos               (31U)
#define RCC_CSR_LPWRRSTF                   (0x1UL << RCC_CSR_LPWRRSTF_Pos)      /*!< 0x80000000 */

/* Legacy defines */
#define RCC_CSR_PADRSTF                    RCC_CSR_PINRSTF
#define RCC_CSR_WDGRSTF                    RCC_CSR_IWDGRSTF

/********************  Bit definition for RCC_SSCGR register  *****************/
#define RCC_SSCGR_MODPER_Pos               (0U)
#define RCC_SSCGR_MODPER                   (0x1FFFUL << RCC_SSCGR_MODPER_Pos)   /*!< 0x00001FFF */

#define RCC_SSCGR_INCSTEP_Pos              (13U)
#define RCC_SSCGR_INCSTEP                  (0x7FFFUL << RCC_SSCGR_INCSTEP_Pos)  /*!< 0x0FFFE000 */

#define RCC_SSCGR_SPREADSEL_Pos            (30U)
#define RCC_SSCGR_SPREADSEL                (0x1UL << RCC_SSCGR_SPREADSEL_Pos)   /*!< 0x40000000 */

#define RCC_SSCGR_SSCGEN_Pos               (31U)
#define RCC_SSCGR_SSCGEN                   (0x1UL << RCC_SSCGR_SSCGEN_Pos)      /*!< 0x80000000 */

/********************  Bit definition for RCC_PLLI2SCFGR register  ************/
#define RCC_PLLI2SCFGR_PLLI2SN_Pos         (6U)
#define RCC_PLLI2SCFGR_PLLI2SN             (0x1FFUL << RCC_PLLI2SCFGR_PLLI2SN_Pos) /*!< 0x00007FC0 */

#define RCC_PLLI2SCFGR_PLLI2SQ_Pos         (24U)
#define RCC_PLLI2SCFGR_PLLI2SQ             (0xFUL << RCC_PLLI2SCFGR_PLLI2SQ_Pos) /*!< 0x0F000000 */

#define RCC_PLLI2SCFGR_PLLI2SR_Pos         (28U)
#define RCC_PLLI2SCFGR_PLLI2SR             (0x7UL << RCC_PLLI2SCFGR_PLLI2SR_Pos) /*!< 0x70000000 */

/********************  Bit definition for RCC_PLLSAICFGR register  ************/
#define RCC_PLLSAICFGR_PLLSAIN_Pos         (6U)
#define RCC_PLLSAICFGR_PLLSAIN             (0x1FFUL << RCC_PLLSAICFGR_PLLSAIN_Pos) /*!< 0x00007FC0 */

#define RCC_PLLSAICFGR_PLLSAIQ_Pos         (24U)
#define RCC_PLLSAICFGR_PLLSAIQ             (0xFUL << RCC_PLLSAICFGR_PLLSAIQ_Pos) /*!< 0x0F000000 */

#define RCC_PLLSAICFGR_PLLSAIR_Pos         (28U)
#define RCC_PLLSAICFGR_PLLSAIR             (0x7UL << RCC_PLLSAICFGR_PLLSAIR_Pos) /*!< 0x70000000 */

/********************  Bit definition for RCC_DCKCFGR register  ***************/
#define RCC_DCKCFGR_PLLI2SDIVQ_Pos        (0U)
#define RCC_DCKCFGR_PLLI2SDIVQ            (0x1FUL << RCC_DCKCFGR_PLLI2SDIVQ_Pos) /*!< 0x0000001F */

#define RCC_DCKCFGR_PLLSAIDIVQ_Pos        (8U)
#define RCC_DCKCFGR_PLLSAIDIVQ            (0x1FUL << RCC_DCKCFGR_PLLSAIDIVQ_Pos) /*!< 0x00001F00 */

#define RCC_DCKCFGR_PLLSAIDIVR_Pos        (16U)
#define RCC_DCKCFGR_PLLSAIDIVR            (0x3UL << RCC_DCKCFGR_PLLSAIDIVR_Pos) /*!< 0x00030000 */

#define RCC_DCKCFGR_SAI1ASRC_Pos           (20U)
#define RCC_DCKCFGR_SAI1ASRC               (0x3UL << RCC_DCKCFGR_SAI1ASRC_Pos)  /*!< 0x00300000 */

#define RCC_DCKCFGR_SAI1BSRC_Pos           (22U)
#define RCC_DCKCFGR_SAI1BSRC               (0x3UL << RCC_DCKCFGR_SAI1BSRC_Pos)  /*!< 0x00C00000 */

#define RCC_DCKCFGR_TIMPRE_Pos             (24U)
#define RCC_DCKCFGR_TIMPRE                 (0x1UL << RCC_DCKCFGR_TIMPRE_Pos)    /*!< 0x01000000 */


#endif /* RCC_PRIVATE_H_ */
