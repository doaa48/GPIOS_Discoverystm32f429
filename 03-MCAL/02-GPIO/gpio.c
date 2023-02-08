/********************************************************************************
 *                                                                              *
 * [FILE NAME]:   gpio.c                                                        *
 *                                                                              *
 * [AUTHOR]:      Ahmed Saeed                                                   *
 *                                                                              *
 * [DATE]:        07/2/2023                                                     *
 *                                                                              *
 * [DESCRIPTION]: Source file for the GPIO driver                               *
 *                                                                              *
 * [VERSION]:     1.0.0                                                         *
 *                                                                              *
 *******************************************************************************/

//#include "gpio_private.h"
#include "gpio.h"
#include "common_macros.h"
#include "RCC.h"


/*===============================================================================
              ##### Initialization and de-initialization functions #####
  ===============================================================================*/

/*
 * Description   : Initializes the GPIOx peripheral according to the specified
 *                 parameters in the GPIO_Init.
 * Parameters    : GPIOx -> where x can be (A..K) to select the GPIO peripheral
 *                 for STM32F429X.
 *
 *                 GPIO_Init -> pointer to a GPIO_InitTypeDef structure that
 *                 contains the configuration information for the specified GPIO
 *                 peripheral.
 * Return values : None.
 * Notes         : None.
 */
void GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init)
{
	uint32 position;
	uint32 ioposition = 0;
	uint32 iocurrent = 0;

	/* Configure the port pins */
	for(position = 0U; position < 16; position++)
	{
		/* Get the IO position */
		ioposition = 1 << position;
		/* Get the current IO position */
		iocurrent = (GPIO_Init->Pin) & ioposition;

		if(iocurrent == ioposition)
		{
			/*--------------------- GPIO Mode Configuration ------------------------*/
			/* In case of Output or Alternate function mode selection */
			if(((GPIO_Init->Mode & 3) == 1) || (GPIO_Init->Mode & 3) == 2)
			{
				/* Configure the IO Speed */
				GPIOx->OSPEEDR = (GPIOx->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED0 << (position * 2))) |
						((GPIO_Init->Speed << (position * 2U)) & (GPIO_OSPEEDR_OSPEED0 << (position * 2)));

				/* Configure the IO Output Type */
				GPIOx->OTYPER = (GPIOx->OTYPER & ~(GPIO_OTYPER_OT0 << position)) |
						((((GPIO_Init->Mode & 16) >> 4) << position) & (GPIO_OTYPER_OT0 << position));
			}

			if((GPIO_Init->Mode & 3) != 3)
			{
				/* Activate the Pull-up or Pull down resistor for the current IO */
				GPIOx->PUPDR = (GPIOx->PUPDR & ~(GPIO_PUPDR_PUPD0 << (position * 2))) |
						(((GPIO_Init->Pull) << (position * 2)) & (GPIO_PUPDR_PUPD0 << (position * 2)));
			}

			/* In case of Alternate function mode selection */
			if((GPIO_Init->Mode & 3) == 2)
			{
				/* Configure Alternate function mapped with the current IO */
				GPIOx->AFR[position >> 3] = (GPIOx->AFR[position >> 3] & ~(0xF << ((uint32)(position & 0x07) * 4))) |
						(((uint32)(GPIO_Init->Alternate) << (((uint32)position & 0x07U) * 4U)) & (0xF << ((uint32)(position & 0x07) * 4)));
			}

			/* Configure IO Direction mode (Input, Output, Alternate or Analog) */
			GPIOx->MODER = (GPIOx->MODER & ~(GPIO_MODER_MODER0 << (position * 2))) |
					(((GPIO_Init->Mode & 3) << (position * 2)) & (GPIO_MODER_MODER0 << (position * 2)));

			/*--------------------- EXTI Mode Configuration ------------------------*/
			/* Configure the External Interrupt or event for the current IO */
			if((GPIO_Init->Mode & 0x30000) != 0)
			{
				/* Enable SYSCFG Clock */
				SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN_Pos);

				/* Delay after an RCC peripheral clock enabling */
				volatile uint32 temp = (RCC->APB2ENR & RCC_APB2ENR_SYSCFGEN);
				(void) (temp);

				SYSCFG->EXTICR[position >> 2] = (SYSCFG->EXTICR[position >> 2] & ~(0x0F << (4 * (position & 0x03)))) |
						(((uint32)(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U))) & (0x0F << (4 * (position & 0x03))));

				/* Clear Rising Falling edge configuration */
				CLEAR_BIT(EXTI->RTSR,iocurrent);

				if((GPIO_Init->Mode & 0x100000) != 0)
				{
					SET_BIT(EXTI->RTSR,iocurrent);
				}

				CLEAR_BIT(EXTI->FTSR,iocurrent);

				if((GPIO_Init->Mode & 0x200000) != 0)
				{
					SET_BIT(EXTI->FTSR,iocurrent);
				}

				CLEAR_BIT(EXTI->EMR,iocurrent);

				if((GPIO_Init->Mode & 0x20000) != 0)
				{
					SET_BIT(EXTI->EMR,iocurrent);
				}

				/* Clear EXTI line configuration */
				CLEAR_BIT(EXTI->IMR,iocurrent);

				if((GPIO_Init->Mode & 0x10000) != 0)
				{
					SET_BIT(EXTI->IMR,iocurrent);
				}
			}
		}
	}
}

/*
 * Description   : De-initializes the GPIOx peripheral registers to their default reset
 *                 values.
 * Parameters    : GPIOx -> where x can be (A..K) to select the GPIO peripheral
 *                 for STM32F429X.
 *
 *                 GPIO_Pin -> specifies the port bit to be written. This
 *                 parameter can be one of GPIO_PIN_x where x can be (0..15).
 * Return values : None.
 * Notes         : None.
 */
void GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32 GPIO_Pin)
{
	uint32 position;
	uint32 ioposition = 0;
	uint32 iocurrent = 0;
	uint32 tmp = 0;

	/* Configure the port pins */
	for(position = 0; position < 16; position++)
	{
		/* Get the IO position */
		ioposition = 1 << position;
		/* Get the current IO position */
		iocurrent = (GPIO_Pin) & ioposition;

		if(iocurrent == ioposition)
		{
			/*------------------------- EXTI Mode Configuration --------------------*/

			tmp = SYSCFG->EXTICR[position >> 2];
			tmp &= (15 << (4 * (position & 3)));
			if(tmp == ((GPIO_GET_INDEX(GPIOx)) << (4 * (position & 3))))
			{
				/* Clear EXTI line configuration */
				CLEAR_BIT(EXTI->IMR,iocurrent);
				CLEAR_BIT(EXTI->EMR,iocurrent);

				/* Clear Rising Falling edge configuration */
				CLEAR_BIT(EXTI->FTSR,iocurrent);
				CLEAR_BIT(EXTI->RTSR,iocurrent);

				/* Configure the External Interrupt or event for the current IO */
				tmp = 15 << (4 * (position & 3));
				SYSCFG->EXTICR[position >> 2] &= ~tmp;
			}

			/*------------------------- GPIO Mode Configuration --------------------*/
			/* Configure IO Direction in Input Floating Mode */
			GPIOx->MODER &= ~(GPIO_MODER_MODER0 << (position * 2));

			/* Configure the default Alternate Function in current IO */
			GPIOx->AFR[position >> 3] &= ~(15 << ((position & 7) * 4)) ;

			/* Deactivate the Pull-up and Pull-down resistor for the current IO */
			GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (position * 2));

			/* Configure the default value IO Output Type */
			GPIOx->OTYPER  &= ~(GPIO_OTYPER_OT0 << position) ;

			/* Configure the default value for IO Speed */
			GPIOx->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (position * 2));
		}
	}
}

/*
 * Description   : Reads the specified input port pin.
 * Parameters    : GPIOx -> where x can be (A..K) to select the GPIO peripheral
 *                 for STM32F429X.
 *
 *                 GPIO_Pin -> specifies the port bit to be written. This
 *                 parameter can be one of GPIO_PIN_x where x can be (0..15).
 * Return values : Reads the specified input port pin.
 * Notes         : None.
 */
uint8 GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16 GPIO_Pin)
{
	if((BIT_IS_SET(GPIOx->IDR,GPIO_Pin)) == 1)
	{
		return 1;
	}

	else
	{
		return 0;
	}
}

/*
 * Description   : Sets or clears the selected data port bit.
 * Parameters    : GPIOx -> where x can be (A..K) to select the GPIO peripheral
 *                 for STM32F429X.
 *
 *                 GPIO_Pin -> specifies the port bit to be written. This
 *                 parameter can be one of GPIO_PIN_x where x can be (0..15).
 *
 *                 PinState -> specifies the value to be written to the selected
 *                 bit. This parameter can be:
 *                 0: to clear the port pin
 *                 1: to set the port pin.
 * Return values : None.
 * Notes         : This function uses GPIOx_BSRR register to allow atomic
 *                 read/modify accesses. In this way, there is no risk of an IRQ
 *                 occurring between the read and the modify access.
 */
void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16 GPIO_Pin, uint8 PinState)
{
	if(PinState == 1)
	{
		SET_BIT(GPIOx->BSRR,GPIO_Pin);
	}

	else
	{
		SET_BIT(GPIOx->BSRR,GPIO_Pin << 16);
	}
}

/*
 * Description   : Toggles the specified GPIO pins.
 * Parameters    : GPIOx -> where x can be (A..K) to select the GPIO peripheral
 *                 for STM32F429X.
 *
 *                 GPIO_Pin -> Specifies the pins to be toggled.
 * Return values : None.
 * Notes         : None.
 */
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16 GPIO_Pin)
{
	/* Set selected pins that were at low level, and reset ones that were high */
	GPIOx->BSRR = (((GPIOx->ODR) & GPIO_Pin) << 16) | (~(GPIOx->ODR) & GPIO_Pin);
}

/*
 * Description   : Locks GPIO Pins configuration registers.
 * Parameters    : GPIOx -> where x can be (A..K) to select the GPIO peripheral
 *                 for STM32F429X.
 *
 *                 GPIO_Pin -> Specifies the pins to be locked.
 * Return values : None.
 * Notes         : The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
 *                 GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.
 *
 *                 The configuration of the locked GPIO pins can no longer be modified
 *                 until the next reset.
 */
GPIO_StatusTypeDef GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16 GPIO_Pin)
{
	volatile uint32 tmp = GPIO_LCKR_LCKK;

	/* Apply lock key write sequence */
	tmp |= GPIO_Pin;
	/* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
	GPIOx->LCKR = tmp;
	/* Reset LCKx bit(s): LCKK='0' + LCK[15-0] */
	GPIOx->LCKR = GPIO_Pin;
	/* Set LCKx bit(s): LCKK='1' + LCK[15-0] */
	GPIOx->LCKR = tmp;
	/* Read LCKR register. This read is mandatory to complete key lock sequence */
	tmp = GPIOx->LCKR;

	/* Read again in order to confirm lock is active */
	if((BIT_IS_SET(GPIOx->LCKR,GPIO_LCKR_LCKK_Pos)) == 1)
	{
		return GPIO_OK;
	}
	else
	{
		return GPIO_ERROR;
	}
}

/*
 * Description   : This function handles EXTI interrupt request.
 * Parameters    : GPIO_Pin -> Specifies the pins connected EXTI line.
 * Return values : None.
 * Notes         : None,
 */
void GPIO_EXTI_IRQHandler(uint16 GPIO_Pin)
{
	/* EXTI line interrupt detected */
	if((BIT_IS_SET(EXTI->PR,GPIO_Pin)) == 1)
	{
		(EXTI->PR = (GPIO_Pin));
		GPIO_EXTI_Callback(GPIO_Pin);
	}
}

/*
 * Description   : EXTI line detection callbacks.
 * Parameters    : GPIO_Pin -> Specifies the pins connected EXTI line.
 * Return values : None.
 * Notes         : None,
 */
void GPIO_EXTI_Callback(uint16 GPIO_Pin)
{
	(void)(GPIO_Pin);
}
