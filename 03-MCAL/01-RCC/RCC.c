/********************************************************************************
 *                                                                              *
 * [FILE NAME]:   RCC.c                                                         *
 *                                                                              *
 * [AUTHOR]:      Ahmed Saeed                                                   *
 *                                                                              *
 * [DATE]:        06/2/2023                                                     *
 *                                                                              *
 * [DESCRIPTION]: Source file for the RCC driver                                *
 *                                                                              *
 * [VERSION]:     1.0.0                                                         *
 *                                                                              *
 *******************************************************************************/

#include "RCC.h"
#include "common_macros.h"
#include "gpio.h"

/*
 * Description :
 * STM32F4xx_System_Private_Variables.
 */
static uint32 SystemCoreClock = 16000000;
static const uint8 AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static const uint8 APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};


/* Private macro -------------------------------------------------------------*/
#define MCO1_GPIO_PORT         GPIOA
#define MCO1_PIN               GPIO_PIN_8

#define MCO2_GPIO_PORT         GPIOC
#define MCO2_PIN               GPIO_PIN_9

/*******************************************************************************
 *                      Functions Definitions                                  *
 *******************************************************************************/

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
void RCC_DeInit(void)
{
	/* Set HSION bit to enable Internal high-speed clock*/
	SET_BIT(RCC->CR,RCC_CR_HSION_Pos);

	/* Reset HSEON, CSSON, PLLON and HSEBYP bits */
	CLEAR_BIT(RCC->CR,RCC_CR_HSEON_Pos);
	CLEAR_BIT(RCC->CR,RCC_CR_CSSON_Pos);
	CLEAR_BIT(RCC->CR,RCC_CR_PLLON_Pos);
	CLEAR_BIT(RCC->CR,RCC_CR_HSEBYP_Pos);

	/* Reset CFGR register */
	RCC->CFGR = 0;

	/* Reset PLLCFGR register */
	RCC->PLLCFGR = 0x24003010;

	/* Disable all interrupts */
	RCC->CIR = 0;
}

/*
 * Description   : Initializes the RCC Oscillators according to the specified
 *                 parameters in the RCC_OscInitTypeDef.
 * Parameters    : RCC_OscInitStruct -> pointer to an RCC_OscInitTypeDef
 *                 structure that contains the configuration information for the
 *                 RCC Oscillators.
 * Return values : RCC status.
 * Notes         : The PLL is not disabled when used as system clock.
 */
RCC_StatusTypeDef RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
	uint32 counter = 0;

	/* Check Null pointer */
	if(RCC_OscInitStruct == NULL_PTR)
	{
		return RCC_ERROR;
	}

	/*------------------------------- HSE Configuration ------------------------*/
	if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
	{
		/* When the HSE is used as system clock or clock source for PLL in these cases HSE will not disabled */
		if(((RCC->CFGR & RCC_CFGR_SWS) == 4) || (((RCC->CFGR & RCC_CFGR_SWS) == 8) && (BIT_IS_SET(RCC->PLLCFGR,RCC_PLLCFGR_PLLSRC_Pos))))
		{
			if((BIT_IS_SET(RCC->CR,RCC_CR_HSERDY_Pos)) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
			{
				return RCC_ERROR;
			}
		}
		else
		{
			/* Set the new HSE configuration ---------------------------------------*/
			if ((RCC_OscInitStruct->HSEState) == RCC_HSE_ON)
			{
				SET_BIT(RCC->CR, RCC_CR_HSEON_Pos);
			}
			else if ((RCC_OscInitStruct->HSEState) == RCC_HSE_BYPASS)
			{
				SET_BIT(RCC->CR, RCC_CR_HSEBYP_Pos);
				SET_BIT(RCC->CR, RCC_CR_HSEON_Pos);
			}
			else if ((RCC_OscInitStruct->HSEState) == RCC_HSE_OFF)
			{
				CLEAR_BIT(RCC->CR, RCC_CR_HSEON_Pos);
				CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP_Pos);
			}

			/* Check the HSE State */
			if((RCC_OscInitStruct->HSEState) != RCC_HSE_OFF)
			{
				/* Wait till HSE is ready */
				while((BIT_IS_SET(RCC->CR,RCC_CR_HSERDY_Pos)) == 0)
				{
					if(counter > HSE_TIMEOUT_VALUE)
					{
						return RCC_TIMEOUT;
					}

					counter++;
				}

				counter = 0;
			}

			else
			{
				/* Wait till HSE is bypassed or disabled */
				while((BIT_IS_SET(RCC->CR,RCC_CR_HSERDY_Pos)) == 1)
				{
					if(counter > HSE_TIMEOUT_VALUE)
					{
						return RCC_TIMEOUT;
					}

					counter++;
				}

				counter = 0;
			}

		}
	}

	/*----------------------------- HSI Configuration --------------------------*/
	if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
	{
		/* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
		if(((RCC->CFGR & RCC_CFGR_SWS) == 0) || (((RCC->CFGR & RCC_CFGR_SWS) == 8) && (BIT_IS_CLEAR(RCC->PLLCFGR,RCC_PLLCFGR_PLLSRC_Pos))))
		{
			/* When HSI is used as system clock it will not disabled */
			if(((BIT_IS_SET(RCC->CR,RCC_CR_HSIRDY_Pos)) == 1) && (RCC_OscInitStruct->HSIState != RCC_HSI_ON))
			{
				return RCC_ERROR;
			}
			/* Otherwise, just the calibration is allowed */
			else
			{
				/* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
				RCC->CR = (RCC->CR & 0xFFFFFF07) | (RCC_OscInitStruct->HSICalibrationValue & 0x000000F8);
			}
		}
		else
		{
			/* Check the HSI State */
			if((RCC_OscInitStruct->HSIState)!= RCC_HSI_OFF)
			{
				/* Enable the Internal High Speed oscillator (HSI). */
				SET_BIT(RCC->CR,RCC_CR_HSION_Pos);

				/* Wait till HSI is ready */
				while((BIT_IS_SET(RCC->CR,RCC_CR_HSIRDY_Pos)) == 0)
				{
					if(counter > HSI_TIMEOUT_VALUE)
					{
						return RCC_TIMEOUT;
					}

					counter++;
				}

				counter = 0;
				/* Adjusts the Internal High Speed oscillator (HSI) calibration value. */
				RCC->CR = (RCC->CR & 0xFFFFFF07) | (RCC_OscInitStruct->HSICalibrationValue & 0x000000F8);
			}
			else
			{
				/* Disable the Internal High Speed oscillator (HSI). */
				CLEAR_BIT(RCC->CR,RCC_CR_HSION_Pos);

				/* Wait till HSI is ready */
				while((BIT_IS_SET(RCC->CR,RCC_CR_HSIRDY_Pos)) == 1)
				{
					if(counter > HSI_TIMEOUT_VALUE)
					{
						return RCC_TIMEOUT;
					}

					counter++;
				}

				counter = 0;
			}
		}
	}

	/*------------------------------ LSI Configuration -------------------------*/
	if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
	{
		/* Check the LSI State */
		if((RCC_OscInitStruct->LSIState)!= RCC_LSI_OFF)
		{
			/* Enable the Internal Low Speed oscillator (LSI). */
			SET_BIT(RCC->CSR,RCC_CSR_LSION_Pos);

			/* Wait till LSI is ready */
			while((BIT_IS_SET(RCC->CR,RCC_CSR_LSIRDY_Pos)) == 0)
			{
				if(counter > LSI_TIMEOUT_VALUE)
				{
					return RCC_TIMEOUT;
				}

				counter++;
			}

			counter = 0;
		}
		else
		{
			/* Disable the Internal Low Speed oscillator (LSI). */
			CLEAR_BIT(RCC->CSR,RCC_CSR_LSION_Pos);

			/* Wait till LSI is ready */
			while((BIT_IS_SET(RCC->CR,RCC_CSR_LSIRDY_Pos)) == 1)
			{
				if(counter > LSI_TIMEOUT_VALUE)
				{
					return RCC_TIMEOUT;
				}

				counter++;
			}

			counter = 0;
		}
	}

	/*------------------------------ LSE Configuration -------------------------*/
	if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
	{
		uint8 pwrclkchanged = 0;

		/* Update LSE configuration in Backup Domain control register    */
		/* Requires to enable write access to Backup Domain of necessary */
		if(((RCC->APB1ENR & (RCC_APB1ENR_PWREN)) == 0))
		{
			SET_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN_Pos);

			/* Delay after an RCC peripheral clock enabling */
			volatile uint32 temp = (RCC->APB1ENR & RCC_APB1ENR_PWREN);
			(void) (temp);

			pwrclkchanged = 1;
		}

		if(BIT_IS_SET(PWR->CR, PWR_CR_DBP_Pos) == 0)
		{
			/* Enable write access to Backup domain */
			SET_BIT(PWR->CR, PWR_CR_DBP_Pos);

			/* Wait for Backup domain Write protection disable */
			while(BIT_IS_SET(PWR->CR, PWR_CR_DBP_Pos) == 0)
			{
				if(counter > RCC_DBP_TIMEOUT_VALUE)
				{
					return RCC_TIMEOUT;
				}

				counter++;
			}

			counter = 0;
		}

		/* Set the new LSE configuration -----------------------------------------*/
		if ((RCC_OscInitStruct->LSEState) == RCC_LSE_ON)
		{
			SET_BIT(RCC->BDCR, RCC_BDCR_LSEON_Pos);
		}
		else if ((RCC_OscInitStruct->LSEState) == RCC_LSE_BYPASS)
		{
			SET_BIT(RCC->BDCR, RCC_BDCR_LSEBYP_Pos);
			SET_BIT(RCC->BDCR, RCC_BDCR_LSEON_Pos);
		}
		else if ((RCC_OscInitStruct->LSEState) == RCC_LSE_OFF)
		{
			CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEON_Pos);
			CLEAR_BIT(RCC->BDCR, RCC_BDCR_LSEBYP_Pos);
		}

		/* Check the LSE State */
		if((RCC_OscInitStruct->LSEState) != RCC_LSE_OFF)
		{
			/* Wait till LSE is ready */
			while((BIT_IS_SET(RCC->CR,RCC_BDCR_LSERDY_Pos)) == 0)
			{
				if(counter > RCC_LSE_TIMEOUT_VALUE)
				{
					return RCC_TIMEOUT;
				}

				counter++;
			}

			counter = 0;
		}
		else
		{
			/* Wait till LSE is ready */
			while((BIT_IS_SET(RCC->CR,RCC_BDCR_LSERDY_Pos)) == 1)
			{
				if(counter > RCC_LSE_TIMEOUT_VALUE)
				{
					return RCC_TIMEOUT;
				}
			}
		}

		/* Restore clock configuration if changed */
		if(pwrclkchanged == 1)
		{
			(RCC->APB1ENR &= ~(RCC_APB1ENR_PWREN_Pos));
		}
	}



	/*-------------------------------- PLL Configuration -----------------------*/
	/* Check the parameters */
	if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
	{
		/* Check if the PLL is used as system clock or not */
		if((RCC->CFGR & RCC_CFGR_SWS) != 8)
		{
			if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
			{
				/* Disable the main PLL. */
				CLEAR_BIT(RCC->CR,RCC_CR_PLLON_Pos);

				/* Wait till PLL is ready */
				while((BIT_IS_SET(RCC->CR,RCC_CR_PLLRDY_Pos)) == 1)
				{
					if(counter > PLL_TIMEOUT_VALUE)
					{
						return RCC_TIMEOUT;
					}

					counter++;
				}

				counter = 0;

				/* Configure the main PLL clock source, multiplication and division factors. */
				RCC->PLLCFGR =  (RCC_OscInitStruct->PLL.PLLSource) |
						(RCC_OscInitStruct->PLL.PLLMF) |
						(RCC_OscInitStruct->PLL.PLLNF << 6) |
						(((RCC_OscInitStruct->PLL.PLLPF >> 1) - 1) << 16) |
						(RCC_OscInitStruct->PLL.PLLQF << 24);
				/* Enable the main PLL. */
				SET_BIT(RCC->CR,RCC_CR_PLLON_Pos);

				/* Wait till PLL is ready */
				while((BIT_IS_SET(RCC->CR,RCC_CR_PLLRDY_Pos)) == 0)
				{
					if(counter > PLL_TIMEOUT_VALUE)
					{
						return RCC_TIMEOUT;
					}

					counter++;
				}

				counter = 0;

			}
			else
			{
				/* Disable the main PLL. */
				CLEAR_BIT(RCC->CR,RCC_CR_PLLON_Pos);

				/* Wait till PLL is ready */
				while((BIT_IS_SET(RCC->CR,RCC_CR_PLLRDY_Pos)) == 1)
				{
					if(counter > PLL_TIMEOUT_VALUE)
					{
						return RCC_TIMEOUT;
					}

					counter++;
				}

				counter = 0;

			}
		}
		else
		{
			/* Check if there is a request to disable the PLL used as System clock source */
			if((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_OFF)
			{
				return RCC_ERROR;
			}
			else
			{
				/* Do not return HAL_ERROR if request repeats the current configuration */
				if (((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_OFF) ||
						((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) != RCC_OscInitStruct->PLL.PLLSource) ||
						((RCC->PLLCFGR & RCC_PLLCFGR_PLLM)   != (RCC_OscInitStruct->PLL.PLLMF) << 0) ||
						((RCC->PLLCFGR & RCC_PLLCFGR_PLLN)   != (RCC_OscInitStruct->PLL.PLLNF) << 6) ||
						((RCC->PLLCFGR & RCC_PLLCFGR_PLLP)   != (((RCC_OscInitStruct->PLL.PLLPF >> 1U) - 1U)) << 16) ||
						((RCC->PLLCFGR & RCC_PLLCFGR_PLLQ)   != (RCC_OscInitStruct->PLL.PLLQF << 24)))
				{
					return RCC_ERROR;
				}
			}
		}
	}
	return RCC_OK;
}



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
RCC_StatusTypeDef RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32 FLatency)
{
	uint32 counter = 0;

	/* Check Null pointer */
	if(RCC_ClkInitStruct == NULL_PTR)
	{
		return RCC_ERROR;
	}

	/* To correctly read data from FLASH memory, the number of wait states (LATENCY)
	     must be correctly programmed according to the frequency of the CPU clock
	     (HCLK) and the supply voltage of the device. */

	/* Increasing the number of wait states because of higher CPU frequency */
	if(FLatency > ((FLASH->ACR) & FLASH_ACR_LATENCY))
	{
		/* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
		(*(volatile uint8 *)(0x40023C00) = (uint8)(FLatency));

		/* Check that the new number of wait states is taken into account to access the Flash
	    memory by reading the FLASH_ACR register */
		if(((FLASH->ACR) & FLASH_ACR_LATENCY) != FLatency)
		{
			return RCC_ERROR;
		}
	}

	/*-------------------------- HCLK Configuration --------------------------*/
	if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
	{
		/* Set the highest APBx dividers in order to ensure that we do not go through
	       a non-spec phase whatever we decrease or increase HCLK. */
		if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
		{
			RCC->CFGR = (RCC->CFGR & 0xFFFFE3FF) | (RCC_HCLK_DIV16 & 0x00001C00);
		}

		if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
		{
			RCC->CFGR = (RCC->CFGR & 0xFFFF1FFF) | ((RCC_HCLK_DIV16 << 3) & 0x0000E000);
		}

		RCC->CFGR = (RCC->CFGR & 0xFFFFFF0F) | ((RCC_ClkInitStruct->AHBCLKDivider) & 0x000000F0);
	}

	/*------------------------- SYSCLK Configuration ---------------------------*/
	if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
	{
		/* HSE is selected as System Clock Source */
		if((RCC_ClkInitStruct->SYSCLKSource) == RCC_SYSCLKSOURCE_HSE)
		{
			/* Check the HSE ready flag */
			if((BIT_IS_SET(RCC->CR,RCC_CR_HSERDY_Pos)) == 0)
			{
				return RCC_ERROR;
			}
		}
		/* PLL is selected as System Clock Source */
		else if((RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK))
		{
			/* Check the PLL ready flag */
			if((BIT_IS_SET(RCC->CR,RCC_CR_PLLRDY_Pos)) == 0)
			{
				return RCC_ERROR;
			}
		}
		/* HSI is selected as System Clock Source */
		else
		{
			/* Check the HSI ready flag */
			if((BIT_IS_SET(RCC->CR,RCC_CR_HSIRDY_Pos)) == 0)
			{
				return RCC_ERROR;
			}
		}

		RCC->CFGR = (RCC->CFGR & 0xFFFFFFFC) | ((RCC_ClkInitStruct->SYSCLKSource) & 0x00000003);

		while ((RCC->CFGR & RCC_CFGR_SWS) != (RCC_ClkInitStruct->SYSCLKSource << 2))
		{
			if (counter > CLOCKSWITCH_TIMEOUT_VALUE)
			{
				return RCC_TIMEOUT;
			}

			counter++;
		}

		counter = 0;
	}

	/* Decreasing the number of wait states because of lower CPU frequency */
	if(FLatency < ((FLASH->ACR) & FLASH_ACR_LATENCY))
	{
		/* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
		(*(volatile uint8 *)(0x40023C00U) = (uint8)(FLatency));

		/* Check that the new number of wait states is taken into account to access the Flash
	    memory by reading the FLASH_ACR register */
		if(((FLASH->ACR) & FLASH_ACR_LATENCY) != FLatency)
		{
			return RCC_ERROR;
		}
	}

	/*-------------------------- PCLK1 Configuration ---------------------------*/
	if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
	{
		RCC->CFGR = (RCC->CFGR & 0xFFFFE3FF) | ((RCC_ClkInitStruct->APB1CLKDivider) & 0x00001C00);
	}

	/*-------------------------- PCLK2 Configuration ---------------------------*/
	if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
	{
		RCC->CFGR = (RCC->CFGR & 0xFFFF3FFF) | (((RCC_ClkInitStruct->APB2CLKDivider) << 3U) & 0x0000E000);
	}

	/* Update the SystemCoreClock global variable */
	SystemCoreClock = RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> 4];

	/* Configure the source of time base considering new system clocks settings */
	//HAL_InitTick(uwTickPrio);

	return RCC_OK;

}


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
void RCC_MCOConfig(uint32 RCC_MCOx, uint32 RCC_MCOSource, uint32 RCC_MCODiv)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	/* RCC_MCO1 */
	if(RCC_MCOx == RCC_MCO1)
	{
		/* MCO1 Clock Enable */
		SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN_Pos);
		/* Delay after an RCC peripheral clock enabling */
		volatile uint32 temp = (RCC->AHB1ENR & RCC_AHB1ENR_GPIOAEN);
		(void) (temp);

		/* Configure the MCO1 pin in alternate function mode */
		GPIO_InitStruct.Pin = MCO1_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
		GPIO_Init(MCO1_GPIO_PORT, &GPIO_InitStruct);

		/* Mask MCO1 and MCO1PRE[2:0] bits then Select MCO1 clock source and prescaler */
		RCC->CFGR = (RCC->CFGR & 0xF89FFFFF) | ((RCC_MCOSource | RCC_MCODiv) & 0x07600000);
	}

	else if(RCC_MCOx == RCC_MCO2)
	{
		/* MCO2 Clock Enable */
		SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);
		/* Delay after an RCC peripheral clock enabling */
		volatile uint32 temp = (RCC->AHB1ENR & RCC_AHB1ENR_GPIOCEN);
		(void) (temp);

		/* Configure the MCO2 pin in alternate function mode */
		GPIO_InitStruct.Pin = MCO2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
		GPIO_Init(MCO2_GPIO_PORT, &GPIO_InitStruct);

		/* Mask MCO2 and MCO2PRE[2:0] bits then Select MCO2 clock source and prescaler */
		RCC->CFGR = (RCC->CFGR & 0x07FFFFFF) | ((RCC_MCOSource | (RCC_MCODiv << 3U)) & 0xF8000000);

	}
}


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
void RCC_EnableCSS(void)
{
	SET_BIT(RCC->CR,RCC_CR_CSSON_Pos);
}


/*
 * Description   : Disables the Clock Security System.
 * Parameters    : None.
 * Return values : None.
 * Notes         : None.
 */
void RCC_DisableCSS(void)
{
	CLEAR_BIT(RCC->CR,RCC_CR_CSSON_Pos);
}


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
uint32 RCC_GetSysClockFreq(void)
{
	uint32 pllm = 0, pllvco = 0, pllp = 0;
	uint32 sysclockfreq = 0;

	/* Get SYSCLK source -------------------------------------------------------*/
	if ((RCC->CFGR & RCC_CFGR_SWS) == 0) /* HSI used as system clock source */
	{
		sysclockfreq = 16000000;
	}

	else if ((RCC->CFGR & RCC_CFGR_SWS) == 4) /* HSE used as system clock  source */
	{
		sysclockfreq = 25000000;
	}

	else if ((RCC->CFGR & RCC_CFGR_SWS) == 8) /* PLL used as system clock  source */
	{
		/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
			      SYSCLK = PLL_VCO / PLLP */
		pllm = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM);
		if((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) != 0)
		{
			/* HSE used as PLL clock source */
			pllvco = (uint32) ((((uint64) 25000000 * ((uint64) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6)))) / (uint64)pllm);
		}
		else
		{
			/* HSI used as PLL clock source */
			pllvco = (uint32) ((((uint64) 16000000 * ((uint64) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6)))) / (uint64)pllm);
		}
		pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16) + 1) *2);

		sysclockfreq = pllvco/pllp;
	}

	else
	{
		sysclockfreq = 16000000;
	}

	return sysclockfreq;
}


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
uint32 RCC_GetHCLKFreq(void)
{
	return SystemCoreClock;
}


/*
 * Description   : Returns the PCLK1 frequency.
 * Parameters    : None.
 * Return values : PCLK1 frequency.
 * Notes         : Each time PCLK1 changes, this function must be called to
 *                 update the right PCLK1 value. Otherwise, any configuration
 *                 based on this function will be incorrect.
 */
uint32 RCC_GetPCLK1Freq(void)
{
	/* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
	return (RCC_GetHCLKFreq()>> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> 10]);
}



/*
 * Description   : Returns the PCLK2 frequency.
 * Parameters    : None.
 * Return values : PCLK2 frequency.
 * Notes         : Each time PCLK2 changes, this function must be called to
 *                 update the right PCLK2 value. Otherwise, any configuration
 *                 based on this function will be incorrect.
 */
uint32 RCC_GetPCLK2Freq(void)
{
	/* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
	return (RCC_GetHCLKFreq()>> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2)>> 13]);
}



/*
 * Description   : Configures the RCC_OscInitStruct according to the internal RCC
 *                 configuration registers.
 * Parameters    : RCC_OscInitStruct -> pointer to an RCC_OscInitTypeDef
 *                 structure that will be configured.
 * Return values : None.
 * Notes         : None.
 */
void RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct)
{
	/* Set all possible values for the Oscillator type parameter ---------------*/
	RCC_OscInitStruct->OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_LSI;

	/* Get the HSE configuration -----------------------------------------------*/
	if(BIT_IS_SET(RCC->CR,RCC_CR_HSEBYP_Pos))
	{
		RCC_OscInitStruct->HSEState = RCC_HSE_BYPASS;
	}
	else if(BIT_IS_SET(RCC->CR,RCC_CR_HSEON_Pos))
	{
		RCC_OscInitStruct->HSEState = RCC_HSE_ON;
	}
	else
	{
		RCC_OscInitStruct->HSEState = RCC_HSE_OFF;
	}

	/* Get the HSI configuration -----------------------------------------------*/
	if(BIT_IS_SET(RCC->CR,RCC_CR_HSION_Pos))
	{
		RCC_OscInitStruct->HSIState = RCC_HSI_ON;
	}
	else
	{
		RCC_OscInitStruct->HSIState = RCC_HSI_OFF;
	}

	RCC_OscInitStruct->HSICalibrationValue = ((RCC->CR & RCC_CR_HSITRIM) >> 3);

	/* Get the LSE configuration -----------------------------------------------*/
	if(BIT_IS_SET(RCC->BDCR,RCC_BDCR_LSEBYP_Pos))
	{
		RCC_OscInitStruct->LSEState = RCC_LSE_BYPASS;
	}
	else if(BIT_IS_SET(RCC->BDCR,RCC_BDCR_LSEON_Pos))
	{
		RCC_OscInitStruct->LSEState = RCC_LSE_ON;
	}
	else
	{
		RCC_OscInitStruct->LSEState = RCC_LSE_OFF;
	}

	/* Get the LSI configuration -----------------------------------------------*/
	if(BIT_IS_SET(RCC->CSR,RCC_CSR_LSION_Pos))
	{
		RCC_OscInitStruct->LSIState = RCC_LSI_ON;
	}
	else
	{
		RCC_OscInitStruct->LSIState = RCC_LSI_OFF;
	}

	/* Get the PLL configuration -----------------------------------------------*/
	if(BIT_IS_SET(RCC->CR,RCC_CR_PLLON_Pos))
	{
		RCC_OscInitStruct->PLL.PLLState = RCC_PLL_ON;
	}
	else
	{
		RCC_OscInitStruct->PLL.PLLState = RCC_PLL_OFF;
	}
	RCC_OscInitStruct->PLL.PLLSource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
	RCC_OscInitStruct->PLL.PLLMF =     (RCC->PLLCFGR & RCC_PLLCFGR_PLLM);
	RCC_OscInitStruct->PLL.PLLNF =     ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
	RCC_OscInitStruct->PLL.PLLPF =     ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) + RCC_PLLCFGR_PLLP_0) << 1) >> 16);
	RCC_OscInitStruct->PLL.PLLQF =     ((RCC->PLLCFGR & RCC_PLLCFGR_PLLQ) >> 24);
}


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
void RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32 *pFLatency)
{
	/* Set all possible values for the Clock type parameter --------------------*/
	RCC_ClkInitStruct->ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

	/* Get the SYSCLK configuration --------------------------------------------*/
	RCC_ClkInitStruct->SYSCLKSource = (RCC->CFGR & RCC_CFGR_SW);

	/* Get the HCLK configuration ----------------------------------------------*/
	RCC_ClkInitStruct->AHBCLKDivider = (RCC->CFGR & RCC_CFGR_HPRE);

	/* Get the APB1 configuration ----------------------------------------------*/
	RCC_ClkInitStruct->APB1CLKDivider = (RCC->CFGR & RCC_CFGR_PPRE1);

	/* Get the APB2 configuration ----------------------------------------------*/
	RCC_ClkInitStruct->APB2CLKDivider = ((RCC->CFGR & RCC_CFGR_PPRE2) >> 3);

	/* Get the Flash Wait State (Latency) configuration ------------------------*/
	*pFLatency = (FLASH->ACR & FLASH_ACR_LATENCY);
}

/*
 * Description   : This function handles the RCC CSS interrupt request.
 * Parameters    : None.
 * Return values : None.
 * Notes         : This API should be called under the NMI_Handler().
 */
void RCC_NMI_IRQHandler(void)
{
	/* Check RCC CSSF flag  */
	if((RCC->CIR & (128)) == (128))
	{
		/* RCC Clock Security System interrupt user callback */
		RCC_CSSCallback();

		/* Clear RCC CSS pending bit */
		(*(volatile uint8 *) RCC_CIR_BYTE2_ADDRESS = 128);
	}
}


/*
 * Description   : RCC Clock Security System interrupt callback.
 * Parameters    : None,
 * Return values : None.
 * Notes         : This function Should not be modified, when the callback is needed,
 *                 the HAL_RCC_CSSCallback could be implemented in the user file.
 */
void RCC_CSSCallback(void)
{

}






