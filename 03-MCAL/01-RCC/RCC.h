/********************************************************************************
 *                                                                              *
 * [FILE NAME]:   RCC.c                                                         *
 *                                                                              *
 * [AUTHOR]:      Ahmed Saeed                                                   *
 *                                                                              *
 * [DATE]:        06/2/2023                                                     *
 *                                                                              *
 * [DESCRIPTION]: Header file for the RCC driver                                *
 *                                                                              *
 * [VERSION]:     1.0.0                                                         *
 *                                                                              *
 *******************************************************************************/

#ifndef RCC_H_
#define RCC_H_


#include "std_types.h"
#include "RCC_private.h"

/*
 * Description :
 * RCC Status structures definition.
 */
typedef enum
{
  RCC_OK,RCC_ERROR,RCC_BUSY,RCC_TIMEOUT
} RCC_StatusTypeDef;



/*
 * Description :
 * RCC PLL configuration structure definition,
 */
typedef struct
{
  uint32 PLLState;   /* The new state of the PLL. */

  uint32 PLLSource;   /* RCC_PLLSource: PLL entry clock source. */

  uint32 PLLMF;       /* PLLM: Division factor for PLL VCO input clock.
                               This parameter must be a number between Min_Data = 0 and Max_Data = 63   */

  uint32 PLLNF;       /* PLLN: Multiplication factor for PLL VCO output clock.
                               This parameter must be a number between Min_Data = 50 and Max_Data = 432 */

  uint32 PLLPF;       /* PLLP: Division factor for main system clock (SYSCLK).  */

  uint32 PLLQF;       /* PLLQ: Division factor for OTG FS, SDIO and RNG clocks.
                               This parameter must be a number between Min_Data = 2 and Max_Data = 15    */
}RCC_PLLInitTypeDef;


/*
 * Description :
 * RCC System, AHB and APB busses clock configuration structure definition
 */
typedef struct
{
  uint32 ClockType;             /* The clock to be configured. */

  uint32 SYSCLKSource;          /* The clock source (SYSCLKS) used as system clock. */

  uint32 AHBCLKDivider;         /* The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK). */

  uint32 APB1CLKDivider;        /* The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK). */

  uint32 APB2CLKDivider;        /* The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK). */

}RCC_ClkInitTypeDef;


/*
 * Description :
 * RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition.
 */
typedef struct
{
  uint32 OscillatorType;       /* The oscillators to be configured. */

  uint32 HSEState;             /* The new state of the HSE. */

  uint32 LSEState;             /* The new state of the LSE. */

  uint32 HSIState;             /* The new state of the HSI. */

  uint32 HSICalibrationValue;  /* The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
                                  This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F  */

  uint32 LSIState;             /* The new state of the LSI. */

  RCC_PLLInitTypeDef PLL;      /* PLL structure parameters  */

}RCC_OscInitTypeDef;


/*
 * Description :
 * PLLI2S Clock structure definition
 */
//typedef struct
//{
//  uint32 PLLI2SN;    /*!< Specifies the multiplication factor for PLLI2S VCO output clock.
//                            This parameter must be a number between Min_Data = 50 and Max_Data = 432.
//                            This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */
//
//  uint32 PLLI2SR;    /*!< Specifies the division factor for I2S clock.
//                            This parameter must be a number between Min_Data = 2 and Max_Data = 7.
//                            This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */
//
//  uint32 PLLI2SQ;    /*!< Specifies the division factor for SAI1 clock.
//                            This parameter must be a number between Min_Data = 2 and Max_Data = 15.
//                            This parameter will be used only when PLLI2S is selected as Clock Source SAI */
//}RCC_PLLI2SInitTypeDef;


/*
 * Description :
 * PLLSAI Clock structure definition
 */
//typedef struct
//{
//  uint32 PLLSAIN;    /*!< Specifies the multiplication factor for PLLI2S VCO output clock.
//                            This parameter must be a number between Min_Data = 50 and Max_Data = 432.
//                            This parameter will be used only when PLLSAI is selected as Clock Source SAI or LTDC */
//
//  uint32 PLLSAIQ;    /*!< Specifies the division factor for SAI1 clock.
//                            This parameter must be a number between Min_Data = 2 and Max_Data = 15.
//                            This parameter will be used only when PLLSAI is selected as Clock Source SAI or LTDC */
//
//  uint32 PLLSAIR;    /*!< specifies the division factor for LTDC clock
//                            This parameter must be a number between Min_Data = 2 and Max_Data = 7.
//                            This parameter will be used only when PLLSAI is selected as Clock Source LTDC */
//
//}RCC_PLLSAIInitTypeDef;

/*
 * Description :
 * RCC extended clocks structure definition.
 */
//typedef struct
//{
//  uint32 PeriphClockSelection; /*!< The Extended Clock to be configured.
//                                      This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */
//
//  RCC_PLLI2SInitTypeDef PLLI2S;  /*!< PLL I2S structure parameters.
//                                      This parameter will be used only when PLLI2S is selected as Clock Source I2S or SAI */
//
//  RCC_PLLSAIInitTypeDef PLLSAI;  /*!< PLL SAI structure parameters.
//                                      This parameter will be used only when PLLI2S is selected as Clock Source SAI or LTDC */
//
//  uint32 PLLI2SDivQ;           /*!< Specifies the PLLI2S division factor for SAI1 clock.
//                                      This parameter must be a number between Min_Data = 1 and Max_Data = 32
//                                      This parameter will be used only when PLLI2S is selected as Clock Source SAI */
//
//  uint32 PLLSAIDivQ;           /*!< Specifies the PLLI2S division factor for SAI1 clock.
//                                      This parameter must be a number between Min_Data = 1 and Max_Data = 32
//                                      This parameter will be used only when PLLSAI is selected as Clock Source SAI */
//
//  uint32 PLLSAIDivR;           /*!< Specifies the PLLSAI division factor for LTDC clock.
//                                      This parameter must be one value of @ref RCCEx_PLLSAI_DIVR */
//
//  uint32 RTCClockSelection;      /*!< Specifies RTC Clock Prescalers Selection.
//                                      This parameter can be a value of @ref RCC_RTC_Clock_Source */
//
//  uint8 TIMPresSelection;      /*!< Specifies TIM Clock Prescalers Selection.
//                                      This parameter can be a value of @ref RCCEx_TIM_PRescaler_Selection */
//
//}RCC_PeriphCLKInitTypeDef;




/* Initialization and de-initialization functions  ******************************/
/*
 * Description   : Resets the RCC clock configuration to the default reset state.
 * Parameters    : None.
 * Return values : None.
 * Notes         : The default reset state of the clock configuration is given
 *                 below: HSI ON and used as system clock sourceHSE, PLL
 *                 and PLLI2S OFFAHB, APB1 and APB2 prescaler set to 1.
 *                 CSS, MCO1 and MCO2 OFFAll interrupts disabled
 *
 *                 This function doesn't modify the configuration of the
 *                 Peripheral clocksLSI, LSE and RTC clocks.
 */
void RCC_DeInit(void);

/*
 * Description   : Initializes the RCC Oscillators according to the specified
 *                 parameters in the RCC_OscInitTypeDef.
 * Parameters    : RCC_OscInitStruct -> pointer to an RCC_OscInitTypeDef
 *                 structure that contains the configuration information for the
 *                 RCC Oscillators.
 * Return values : RCC status.
 * Notes         : The PLL is not disabled when used as system clock.
 */
RCC_StatusTypeDef RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);

/*
 * Description   : Initializes the CPU, AHB and APB busses clocks according to the
 *                 specified parameters in the RCC_ClkInitStruct.
 * Parameters    : RCC_ClkInitStruct -> pointer to an RCC_OscInitTypeDef
 *                 structure that contains the configuration information for the
 *                 RCC peripheral.
 *
 *                 FLatency -> FLASH Latency, this parameter depend on
 *                 device selected.
 * Return values : RCC status.
 * Notes         : The SystemCoreClock CMSIS variable is used to store
 *                 System Clock Frequency and updated by
 *                 HAL_RCC_GetHCLKFreq() function called within this function.
 *
 *                 The HSI is used (enabled by hardware) as system clock
 *                 source after startup from Reset, wake-up from STOP and
 *                 STANDBY mode, or in case of failure of the HSE used
 *                 directly or indirectly as system clock (if the Clock Security
 *                 System CSS is enabled).
 *
 *                 A switch from one clock source to another occurs only if the
 *                 target clock source is ready (clock stable after startup delay or
 *                 PLL locked). If a clock source which is not yet ready is
 *                 selected, the switch will occur when the clock source will be ready.
 *
 *                 Depending on the device voltage range, the software has to
 *                 set correctly HPRE[3:0] bits to ensure that HCLK not exceed
 *                 the maximum allowed frequency.
 */
RCC_StatusTypeDef RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32 FLatency);


/* Peripheral Control functions  ************************************************/
/*
 * Description   : Selects the clock source to output on MCO1 pin(PA8) or on
 *                 MCO2 pin(PC9).
 * Parameters    : RCC_MCOx -> specifies the output direction for the clock
 *                 source. This parameter can be one of the following values:
 *                 - RCC_MCO1 : Clock source to output on MCO1 pin (PA8).
 *                 - RCC_MCO2 : Clock source to output on MCO2 pin (PC9).
 *
 *                 RCC_MCOSource -> specifies the clock source to output.
 *                 This parameter can be one of the following values:
 *                 - RCC_MCO1SOURCE_HSI : HSI clock selected as MCO1 source.
 *                 - RCC_MCO1SOURCE_LSE : LSE clock selected as MCO1 source.
 *                 - RCC_MCO1SOURCE_HSE : HSE clock selected as MCO1 source.
 *                 - RCC_MCO1SOURCE_PLLCLK : main PLL clock selected as MCO1 source.
 *                 - RCC_MCO2SOURCE_SYSCLK : System clock (SYSCLK) selected as MCO2 source.
 *                 - RCC_MCO2SOURCE_PLLI2SCLK : PLLI2S clock selected as MCO2 source.
 *                 - RCC_MCO2SOURCE_HSE : HSE clock selected as MCO2 source.
 *                 - RCC_MCO2SOURCE_PLLCLK : main PLL clock selected as MCO2 source.
 *
 *                 RCC_MCODiv -> specifies the MCOx prescaler. This
 *                 parameter can be one of the following values:
 *                 - RCC_MCODIV_1 : no division applied to MCOx clock.
 *                 - RCC_MCODIV_2 : division by 2 applied to MCOx clock.
 *                 - RCC_MCODIV_3 : division by 3 applied to MCOx clock.
 *                 - RCC_MCODIV_4 : division by 4 applied to MCOx clock.
 *                 - RCC_MCODIV_5 : division by 5 applied to MCOx clock.
 * Return values : None.
 * Notes         : PA8/PC9 should be configured in alternate function mode.
 */
void     RCC_MCOConfig(uint32 RCC_MCOx, uint32 RCC_MCOSource, uint32 RCC_MCODiv);

/*
 * Description   : Enables the Clock Security System.
 * Parameters    : None.
 * Return values : None.
 * Notes         : If a failure is detected on the HSE oscillator clock, this
 *                 oscillator is automatically disabled and an interrupt is
 *                 generated to inform the software about the failure (Clock
 *                 Security System Interrupt, CSSI), allowing the MCU to
 *                 perform rescue operations. The CSSI is linked to the Cortex-M4
 *                 NMI (Non-Maskable Interrupt) exception vector.
 */
void     RCC_EnableCSS(void);

/*
 * Description   : Disables the Clock Security System.
 * Parameters    : None.
 * Return values : None.
 * Notes         : None.
 */
void     RCC_DisableCSS(void);

/*
 * Description   : Returns the SYSCLK frequency.
 * Parameters    : None.
 * Return values : SYSCLK frequency.
 * Notes         : The system frequency computed by this function is not the
 *                 real frequency in the chip. It is calculated based on the
 *                 predefined constant and the selected clock source:
 *                 - If SYSCLK source is HSI, function returns values based on
 *                 HSI_VALUE.
 *                 - If SYSCLK source is HSE, function returns values based on
 *                 HSE_VALUE
 *                 - If SYSCLK source is PLL, function returns values based on
 *                 HSE_VALUE or HSI_VALUE multiplied/divided by the PLL factors.
 *                 - HSI_VALUE is a constant (default value 16 MHz) but the real
 *                 value may vary depending on the variations in voltage
 *                 and temperature.
 *                 -  HSE_VALUE is a constant (default value 25 MHz), user has to
 *                 ensure that HSE_VALUE is same as the real frequency of the
 *                 crystal used. Otherwise, this function may have wrong result.
 *                 - The result of this function could be not correct when using
 *                 fractional value for HSE crystal.
 *                 - This function can be used by the user application to compute
 *                 the baudrate for the communication peripherals or configure
 *                 other parameters.
 *                 - Each time SYSCLK changes, this function must be called to
 *                 update the right SYSCLK value. Otherwise, any configuration
 *                 based on this function will be incorrect.
 */
uint32   RCC_GetSysClockFreq(void);

/*
 * Description   : Returns the HCLK frequency.
 * Parameters    : None.
 * Return values : HCLK frequency.
 * Notes         : Each time HCLK changes, this function must be called to
 *                 update the right HCLK value. Otherwise, any configuration
 *                 based on this function will be incorrect.
 *
 *                 The SystemCoreClock CMSIS variable is used to store
 *                 System Clock Frequency and updated within this function.
 */
uint32   RCC_GetHCLKFreq(void);

/*
 * Description   : Returns the PCLK1 frequency.
 * Parameters    : None.
 * Return values : PCLK1 frequency.
 * Notes         : Each time PCLK1 changes, this function must be called to
 *                 update the right PCLK1 value. Otherwise, any configuration
 *                 based on this function will be incorrect.
 */
uint32   RCC_GetPCLK1Freq(void);

/*
 * Description   : Returns the PCLK2 frequency.
 * Parameters    : None.
 * Return values : PCLK2 frequency.
 * Notes         : Each time PCLK2 changes, this function must be called to
 *                 update the right PCLK2 value. Otherwise, any configuration
 *                 based on this function will be incorrect.
 */
uint32   RCC_GetPCLK2Freq(void);

/*
 * Description   : Configures the RCC_OscInitStruct according to the internal RCC
 *                 configuration registers.
 * Parameters    : RCC_OscInitStruct -> pointer to an RCC_OscInitTypeDef
 *                 structure that will be configured.
 * Return values : None.
 * Notes         : None.
 */
void     RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);

/*
 * Description   : Configures the RCC_ClkInitStruct according to the internal RCC
 *                 configuration registers.
 * Parameters    : RCC_OscInitStruct -> pointer to an RCC_ClkInitTypeDef
 *                 structure that will be configured.
 *
 *                 pFLatency -> Pointer on the Flash Latency.
 * Return values : None.
 * Notes         : None.
 */
void     RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32 *pFLatency);

/* CSS NMI IRQ handler */
/*
 * Description   : This function handles the RCC CSS interrupt request.
 * Parameters    : None.
 * Return values : None.
 * Notes         : This API should be called under the NMI_Handler().
 */
void RCC_NMI_IRQHandler(void);

/* User Callbacks in non blocking mode (IT mode) */
/*
 * Description   : RCC Clock Security System interrupt callback.
 * Parameters    : None,
 * Return values : None.
 * Notes         : This function Should not be modified, when the callback is needed,
 *                 the HAL_RCC_CSSCallback could be implemented in the user file.
 */
void RCC_CSSCallback(void);


/*
 * Description :
 * RCC oscillators timeout.
 */
#define PLL_TIMEOUT_VALUE          2U  /* 2 ms */
#define LSI_TIMEOUT_VALUE          2U  /* 2 ms */
#define HSI_TIMEOUT_VALUE          2U  /* 2 ms */
#define HSE_TIMEOUT_VALUE          HSE_STARTUP_TIMEOUT
#define HSE_STARTUP_TIMEOUT    100U   /*!< Time out for HSE start up, in ms */
#define LSE_STARTUP_TIMEOUT    5000U   /*!< Time out for LSE start up, in ms */

/*
 * Description :
 * RCC_AHB_Clock_Source AHB Clock Source.
 */
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512

/*
 * Description :
 * RCC_APB1_APB2_Clock_Source APB1/APB2 Clock Source.
 */
#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1
#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4
#define RCC_HCLK_DIV8                    RCC_CFGR_PPRE1_DIV8
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16


/*
 * Description :
 * RCC registers bit address in the alias region.
 */
#define RCC_OFFSET                 (RCC_BASE - PERIPH_BASE)
/* --- CR Register --- */
/* Alias word address of HSION bit */
#define RCC_CR_OFFSET              (RCC_OFFSET + 0x00U)
#define RCC_HSION_BIT_NUMBER       0x00U
#define RCC_CR_HSION_BB            (PERIPH_BB_BASE + (RCC_CR_OFFSET * 32U) + (RCC_HSION_BIT_NUMBER * 4U))
/* Alias word address of CSSON bit */
#define RCC_CSSON_BIT_NUMBER       0x13U
#define RCC_CR_CSSON_BB            (PERIPH_BB_BASE + (RCC_CR_OFFSET * 32U) + (RCC_CSSON_BIT_NUMBER * 4U))
/* Alias word address of PLLON bit */
#define RCC_PLLON_BIT_NUMBER       0x18U
#define RCC_CR_PLLON_BB            (PERIPH_BB_BASE + (RCC_CR_OFFSET * 32U) + (RCC_PLLON_BIT_NUMBER * 4U))
/* Alias word address of PLLI2SON bit */
#define RCC_PLLI2SON_BIT_NUMBER    0x1AU
#define RCC_CR_PLLI2SON_BB         (PERIPH_BB_BASE + (RCC_CR_OFFSET * 32U) + (RCC_PLLI2SON_BIT_NUMBER * 4U))
/* --- CFGR Register ---*/
#define RCC_CFGR_OFFSET            (RCC_OFFSET + 0x08U)
/* Alias word address of I2SSRC bit */
#define RCC_I2SSRC_BIT_NUMBER      0x17U
#define RCC_CFGR_I2SSRC_BB         (PERIPH_BB_BASE + (RCC_CFGR_OFFSET * 32U) + (RCC_I2SSRC_BIT_NUMBER * 4U))
/* --- BDCR Register --- */
/* Alias word address of RTCEN bit */
#define RCC_BDCR_OFFSET            (RCC_OFFSET + 0x70U)
#define RCC_RTCEN_BIT_NUMBER       0x0FU
#define RCC_BDCR_RTCEN_BB          (PERIPH_BB_BASE + (RCC_BDCR_OFFSET * 32U) + (RCC_RTCEN_BIT_NUMBER * 4U))
/* Alias word address of BDRST bit */
#define RCC_BDRST_BIT_NUMBER       0x10U
#define RCC_BDCR_BDRST_BB          (PERIPH_BB_BASE + (RCC_BDCR_OFFSET * 32U) + (RCC_BDRST_BIT_NUMBER * 4U))

/* --- CSR Register --- */
/* Alias word address of LSION bit */
#define RCC_CSR_OFFSET             (RCC_OFFSET + 0x74U)
#define RCC_LSION_BIT_NUMBER        0x00U
#define RCC_CSR_LSION_BB           (PERIPH_BB_BASE + (RCC_CSR_OFFSET * 32U) + (RCC_LSION_BIT_NUMBER * 4U))

/* CR register byte 3 (Bits[23:16]) base address */
#define RCC_CR_BYTE2_ADDRESS       0x40023802U

/* CIR register byte 2 (Bits[15:8]) base address */
#define RCC_CIR_BYTE1_ADDRESS      ((uint32)(RCC_BASE + 0x0CU + 0x01U))

/* CIR register byte 3 (Bits[23:16]) base address */
#define RCC_CIR_BYTE2_ADDRESS      ((uint32)(RCC_BASE + 0x0CU + 0x02U))

/* BDCR register base address */
#define RCC_BDCR_BYTE0_ADDRESS     (PERIPH_BASE + RCC_BDCR_OFFSET)
#define CLOCKSWITCH_TIMEOUT_VALUE  5000U /* 5 s */

#define RCC_DBP_TIMEOUT_VALUE      2U
#define RCC_LSE_TIMEOUT_VALUE      LSE_STARTUP_TIMEOUT
/* Flags in the CR register */
#define RCC_FLAG_HSIRDY                  ((uint8)0x21)
#define RCC_FLAG_HSERDY                  ((uint8)0x31)
#define RCC_FLAG_PLLRDY                  ((uint8)0x39)
#define RCC_FLAG_PLLI2SRDY               ((uint8)0x3B)

/* Flags in the BDCR register */
#define RCC_FLAG_LSERDY                  ((uint8)0x41)

/* Flags in the CSR register */
#define RCC_FLAG_LSIRDY                  ((uint8)0x61)
#define RCC_FLAG_BORRST                  ((uint8)0x79)
#define RCC_FLAG_PINRST                  ((uint8)0x7A)
#define RCC_FLAG_PORRST                  ((uint8)0x7B)
#define RCC_FLAG_SFTRST                  ((uint8)0x7C)
#define RCC_FLAG_IWDGRST                 ((uint8)0x7D)
#define RCC_FLAG_WWDGRST                 ((uint8)0x7E)
#define RCC_FLAG_LPWRRST                 ((uint8)0x7F)
/*
 * Description :
 * RCC_HSE_Config HSE Config.
 */
#define RCC_HSE_OFF                      0x00000000U
#define RCC_HSE_ON                       RCC_CR_HSEON
#define RCC_HSE_BYPASS                   ((uint32)(RCC_CR_HSEBYP | RCC_CR_HSEON))
/*
 * Description :
 * RCC_HSI_Config HSI Config.
 */
#define RCC_HSI_OFF                      ((uint8)0x00)
#define RCC_HSI_ON                       ((uint8)0x01)
/*
 * Description :
 * RCCEx_I2S_Clock_Source I2S Clock Source.
 */
#define RCC_I2SCLKSOURCE_PLLI2S         0x00000000U
#define RCC_I2SCLKSOURCE_EXT            0x00000001U
/*
 * Description :
 * RCC_Interrupt Interrupts.
 */
#define RCC_IT_LSIRDY                    ((uint8)0x01)
#define RCC_IT_LSERDY                    ((uint8)0x02)
#define RCC_IT_HSIRDY                    ((uint8)0x04)
#define RCC_IT_HSERDY                    ((uint8)0x08)
#define RCC_IT_PLLRDY                    ((uint8)0x10)
#define RCC_IT_PLLI2SRDY                 ((uint8)0x20)
#define RCC_IT_CSS                       ((uint8)0x80)
/*
 * Description :
 * RCC_LSE_Config LSE Config.
 */
#define RCC_LSE_OFF                    0x00000000U
#define RCC_LSE_ON                     RCC_BDCR_LSEON
#define RCC_LSE_BYPASS                 ((uint32)(RCC_BDCR_LSEBYP | RCC_BDCR_LSEON))
/*
 * Description :
 * RCC_LSI_Config LSI Config.
 */
#define RCC_LSI_OFF                      ((uint8)0x00)
#define RCC_LSI_ON                       ((uint8)0x01)
/*
 * Description :
 * RCC_MCO1_Clock_Source MCO1 Clock Source.
 */
#define RCC_MCO1SOURCE_HSI               0x00000000U
#define RCC_MCO1SOURCE_LSE               RCC_CFGR_MCO1_0
#define RCC_MCO1SOURCE_HSE               RCC_CFGR_MCO1_1
#define RCC_MCO1SOURCE_PLLCLK            RCC_CFGR_MCO1
/*
 * Description :
 * RCC_MCO2_Clock_Source MCO2 Clock Source.
 */
#define RCC_MCO2SOURCE_SYSCLK            0x00000000U
#define RCC_MCO2SOURCE_PLLI2SCLK         RCC_CFGR_MCO2_0
#define RCC_MCO2SOURCE_HSE               RCC_CFGR_MCO2_1
#define RCC_MCO2SOURCE_PLLCLK            RCC_CFGR_MCO2
/*
 * Description :
 * RCC_MCOx_Clock_Prescaler MCOx Clock Prescaler.
 */
#define RCC_MCODIV_1                    0x00000000U
#define RCC_MCODIV_2                    RCC_CFGR_MCO1PRE_2
#define RCC_MCODIV_3                    ((uint32)RCC_CFGR_MCO1PRE_0 | RCC_CFGR_MCO1PRE_2)
#define RCC_MCODIV_4                    ((uint32)RCC_CFGR_MCO1PRE_1 | RCC_CFGR_MCO1PRE_2)
#define RCC_MCODIV_5                    RCC_CFGR_MCO1PRE
/*
 * Description :
 * RCC_MCO_Index MCO Index.
 */
#define RCC_MCO1                         0x00000000U
#define RCC_MCO2                         0x00000001U
/*
 * Description :
 * RCC_Oscillator_Type Oscillator Type.
 */
#define RCC_OSCILLATORTYPE_NONE            0x00000000U
#define RCC_OSCILLATORTYPE_HSE             0x00000001U
#define RCC_OSCILLATORTYPE_HSI             0x00000002U
#define RCC_OSCILLATORTYPE_LSE             0x00000004U
#define RCC_OSCILLATORTYPE_LSI             0x00000008U
/*
 * Description :
 * RCC_PLLP_Clock_Divider PLLP Clock Divider.
 */
#define RCC_PLLP_DIV2                  0x00000002U
#define RCC_PLLP_DIV4                  0x00000004U
#define RCC_PLLP_DIV6                  0x00000006U
#define RCC_PLLP_DIV8                  0x00000008U
/*
 * Description :
 * RCC_PLL_Clock_Source PLL Clock Source.
 */
#define RCC_PLLSOURCE_HSI                RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLLSOURCE_HSE                RCC_PLLCFGR_PLLSRC_HSE
/*
 * Description :
 * RCC_PLL_Config PLL Config.
 */
#define RCC_PLL_NONE                      ((uint8)0x00)
#define RCC_PLL_OFF                       ((uint8)0x01)
#define RCC_PLL_ON                        ((uint8)0x02)
/*
 * Description :
 * RCC_RTC_Clock_Source RTC Clock Source.
 */
#define RCC_RTCCLKSOURCE_NO_CLK          0x00000000U
#define RCC_RTCCLKSOURCE_LSE             0x00000100U
#define RCC_RTCCLKSOURCE_LSI             0x00000200U
#define RCC_RTCCLKSOURCE_HSE_DIVX        0x00000300U
#define RCC_RTCCLKSOURCE_HSE_DIV2        0x00020300U
#define RCC_RTCCLKSOURCE_HSE_DIV3        0x00030300U
#define RCC_RTCCLKSOURCE_HSE_DIV4        0x00040300U
#define RCC_RTCCLKSOURCE_HSE_DIV5        0x00050300U
#define RCC_RTCCLKSOURCE_HSE_DIV6        0x00060300U
#define RCC_RTCCLKSOURCE_HSE_DIV7        0x00070300U
#define RCC_RTCCLKSOURCE_HSE_DIV8        0x00080300U
#define RCC_RTCCLKSOURCE_HSE_DIV9        0x00090300U
#define RCC_RTCCLKSOURCE_HSE_DIV10       0x000A0300U
#define RCC_RTCCLKSOURCE_HSE_DIV11       0x000B0300U
#define RCC_RTCCLKSOURCE_HSE_DIV12       0x000C0300U
#define RCC_RTCCLKSOURCE_HSE_DIV13       0x000D0300U
#define RCC_RTCCLKSOURCE_HSE_DIV14       0x000E0300U
#define RCC_RTCCLKSOURCE_HSE_DIV15       0x000F0300U
#define RCC_RTCCLKSOURCE_HSE_DIV16       0x00100300U
#define RCC_RTCCLKSOURCE_HSE_DIV17       0x00110300U
#define RCC_RTCCLKSOURCE_HSE_DIV18       0x00120300U
#define RCC_RTCCLKSOURCE_HSE_DIV19       0x00130300U
#define RCC_RTCCLKSOURCE_HSE_DIV20       0x00140300U
#define RCC_RTCCLKSOURCE_HSE_DIV21       0x00150300U
#define RCC_RTCCLKSOURCE_HSE_DIV22       0x00160300U
#define RCC_RTCCLKSOURCE_HSE_DIV23       0x00170300U
#define RCC_RTCCLKSOURCE_HSE_DIV24       0x00180300U
#define RCC_RTCCLKSOURCE_HSE_DIV25       0x00190300U
#define RCC_RTCCLKSOURCE_HSE_DIV26       0x001A0300U
#define RCC_RTCCLKSOURCE_HSE_DIV27       0x001B0300U
#define RCC_RTCCLKSOURCE_HSE_DIV28       0x001C0300U
#define RCC_RTCCLKSOURCE_HSE_DIV29       0x001D0300U
#define RCC_RTCCLKSOURCE_HSE_DIV30       0x001E0300U
#define RCC_RTCCLKSOURCE_HSE_DIV31       0x001F0300U
/*
 * Description :
 * RCC_System_Clock_Source System Clock Source.
 */
#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL
/*
 * Description :
 * RCC_System_Clock_Type System Clock Type.
 */
#define RCC_CLOCKTYPE_SYSCLK             0x00000001U
#define RCC_CLOCKTYPE_HCLK               0x00000002U
#define RCC_CLOCKTYPE_PCLK1              0x00000004U
#define RCC_CLOCKTYPE_PCLK2              0x00000008U



#endif /* RCC_H_ */
