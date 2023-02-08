/********************************************************************************
 *                                                                              *
 * [FILE NAME]:   gpio.h                                                        *
 *                                                                              *
 * [AUTHOR]:      Ahmed Saeed                                                   *
 *                                                                              *
 * [DATE]:        07/2/2023                                                     *
 *                                                                              *
 * [DESCRIPTION]: Header file for the GPIO driver                               *
 *                                                                              *
 * [VERSION]:     1.0.0                                                         *
 *                                                                              *
 *******************************************************************************/

#ifndef GPIO_H_
#define GPIO_H_

#include "std_types.h"
#include "gpio_private.h"


/*
 * Description :
 * GPIO Status structures definition.
 */
typedef enum
{
  GPIO_OK,GPIO_ERROR,GPIO_BUSY,GPIO_TIMEOUT
} GPIO_StatusTypeDef;



/*
 * Description :
 * GPIO Init structure definition.
 */
typedef struct
{
  uint32 Pin;        /* Specifies the GPIO pins to be configured. */

  uint32 Mode;       /* Specifies the operating mode for the selected pins. */

  uint32 Pull;       /* Specifies the Pull-up or Pull-Down activation for the selected pins. */

  uint32 Speed;      /* Specifies the speed for the selected pins. */

  uint32 Alternate;  /* Peripheral to be connected to the selected pins. */

}GPIO_InitTypeDef;


/*
 * Description :
 * GPIO pins define.
 */
#define GPIO_PIN_0                 ((uint16)0x0001)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint16)0x0002)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint16)0x0004)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint16)0x0008)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint16)0x0010)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint16)0x0020)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint16)0x0040)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint16)0x0080)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint16)0x0100)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint16)0x0200)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint16)0x0400)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint16)0x0800)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint16)0x1000)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint16)0x2000)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint16)0x4000)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint16)0x8000)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint16)0xFFFF)  /* All pins selected */


/*
 * Description :
 * GPIO Configuration Mode.
 */
#define  GPIO_MODE_INPUT                        MODE_INPUT                                                  /*!< Input Floating Mode                   */
#define  GPIO_MODE_OUTPUT_PP                    (MODE_OUTPUT | OUTPUT_PP)                                   /*!< Output Push Pull Mode                 */
#define  GPIO_MODE_OUTPUT_OD                    (MODE_OUTPUT | OUTPUT_OD)                                   /*!< Output Open Drain Mode                */
#define  GPIO_MODE_AF_PP                        (MODE_AF | OUTPUT_PP)                                       /*!< Alternate Function Push Pull Mode     */
#define  GPIO_MODE_AF_OD                        (MODE_AF | OUTPUT_OD)                                       /*!< Alternate Function Open Drain Mode    */

#define  GPIO_MODE_ANALOG                       MODE_ANALOG                                                 /*!< Analog Mode  */

#define  GPIO_MODE_IT_RISING                    (MODE_INPUT | EXTI_IT | TRIGGER_RISING)                     /*!< External Interrupt Mode with Rising edge trigger detection          */
#define  GPIO_MODE_IT_FALLING                   (MODE_INPUT | EXTI_IT | TRIGGER_FALLING)                    /*!< External Interrupt Mode with Falling edge trigger detection         */
#define  GPIO_MODE_IT_RISING_FALLING            (MODE_INPUT | EXTI_IT | TRIGGER_RISING | TRIGGER_FALLING)   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

#define  GPIO_MODE_EVT_RISING                   (MODE_INPUT | EXTI_EVT | TRIGGER_RISING)                     /*!< External Event Mode with Rising edge trigger detection             */
#define  GPIO_MODE_EVT_FALLING                  (MODE_INPUT | EXTI_EVT | TRIGGER_FALLING)                    /*!< External Event Mode with Falling edge trigger detection            */
#define  GPIO_MODE_EVT_RISING_FALLING           (MODE_INPUT | EXTI_EVT | TRIGGER_RISING | TRIGGER_FALLING)   /*!< External Event Mode with Rising/Falling edge trigger detection     */

/*
 * Description :
 * GPIO Output Maximum frequency.
 */
#define  GPIO_SPEED_FREQ_LOW         0x00000000U  /*!< IO works at 2 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_MEDIUM      0x00000001U  /*!< range 12,5 MHz to 50 MHz, please refer to the product datasheet */
#define  GPIO_SPEED_FREQ_HIGH        0x00000002U  /*!< range 25 MHz to 100 MHz, please refer to the product datasheet  */
#define  GPIO_SPEED_FREQ_VERY_HIGH   0x00000003U  /*!< range 50 MHz to 200 MHz, please refer to the product datasheet  */


/*
 * Description :
 * GPIO Pull-Up or Pull-Down Activation.
 */
#define  GPIO_NOPULL        0x00000000U   /*!< No Pull-up or Pull-down activation  */
#define  GPIO_PULLUP        0x00000001U   /*!< Pull-up activation                  */
#define  GPIO_PULLDOWN      0x00000002U   /*!< Pull-down activation                */


/*
 * Description :
 * GPIO Index.
 */
#define GPIO_GET_INDEX(__GPIOx__)    (uint8)  (((__GPIOx__) == (GPIOA))? 0U :\
                                               ((__GPIOx__) == (GPIOB))? 1U :\
                                               ((__GPIOx__) == (GPIOC))? 2U :\
                                               ((__GPIOx__) == (GPIOD))? 3U :\
                                               ((__GPIOx__) == (GPIOE))? 4U :\
                                               ((__GPIOx__) == (GPIOF))? 5U :\
                                               ((__GPIOx__) == (GPIOG))? 6U :\
                                               ((__GPIOx__) == (GPIOH))? 7U :\
                                               ((__GPIOx__) == (GPIOI))? 8U :\
                                               ((__GPIOx__) == (GPIOJ))? 9U : 10U)



/* Initialization and de-initialization functions *****************************/
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
void  GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);

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
void  GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32 GPIO_Pin);

/* IO operation functions *****************************************************/
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
uint8 GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16 GPIO_Pin);

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
void GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16 GPIO_Pin, uint8 PinState);

/*
 * Description   : Toggles the specified GPIO pins.
 * Parameters    : GPIOx -> where x can be (A..K) to select the GPIO peripheral
 *                 for STM32F429X.
 *
 *                 GPIO_Pin -> Specifies the pins to be toggled.
 * Return values : None.
 * Notes         : None.
 */
void GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16 GPIO_Pin);

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
GPIO_StatusTypeDef GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16 GPIO_Pin);

/*
 * Description   : This function handles EXTI interrupt request.
 * Parameters    : GPIO_Pin -> Specifies the pins connected EXTI line.
 * Return values : None.
 * Notes         : None,
 */
void GPIO_EXTI_IRQHandler(uint16 GPIO_Pin);

/*
 * Description   : EXTI line detection callbacks.
 * Parameters    : GPIO_Pin -> Specifies the pins connected EXTI line.
 * Return values : None.
 * Notes         : None,
 */
void GPIO_EXTI_Callback(uint16 GPIO_Pin);


/*
 * Description :
 * GPIO Private Constants.
 */
#define GPIO_MODE_Pos                           0U
#define GPIO_MODE                               (0x3UL << GPIO_MODE_Pos)
#define MODE_INPUT                              (0x0UL << GPIO_MODE_Pos)
#define MODE_OUTPUT                             (0x1UL << GPIO_MODE_Pos)
#define MODE_AF                                 (0x2UL << GPIO_MODE_Pos)
#define MODE_ANALOG                             (0x3UL << GPIO_MODE_Pos)
#define OUTPUT_TYPE_Pos                         4U
#define OUTPUT_TYPE                             (0x1UL << OUTPUT_TYPE_Pos)
#define OUTPUT_PP                               (0x0UL << OUTPUT_TYPE_Pos)
#define OUTPUT_OD                               (0x1UL << OUTPUT_TYPE_Pos)
#define EXTI_MODE_Pos                           16U
#define EXTI_MODE                               (0x3UL << EXTI_MODE_Pos)
#define EXTI_IT                                 (0x1UL << EXTI_MODE_Pos)
#define EXTI_EVT                                (0x2UL << EXTI_MODE_Pos)
#define TRIGGER_MODE_Pos                         20U
#define TRIGGER_MODE                            (0x7UL << TRIGGER_MODE_Pos)
#define TRIGGER_RISING                          (0x1UL << TRIGGER_MODE_Pos)
#define TRIGGER_FALLING                         (0x2UL << TRIGGER_MODE_Pos)


/*
 * Description :
 * AF 0 selection.
 */
#define GPIO_AF0_RTC_50Hz      ((uint8)0x00)  /* RTC_50Hz Alternate Function mapping                       */
#define GPIO_AF0_MCO           ((uint8)0x00)  /* MCO (MCO1 and MCO2) Alternate Function mapping            */
#define GPIO_AF0_TAMPER        ((uint8)0x00)  /* TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
#define GPIO_AF0_SWJ           ((uint8)0x00)  /* SWJ (SWD and JTAG) Alternate Function mapping             */
#define GPIO_AF0_TRACE         ((uint8)0x00)  /* TRACE Alternate Function mapping                          */

/*
 * Description :
 * AF 1 selection.
 */
#define GPIO_AF1_TIM1          ((uint8)0x01)  /* TIM1 Alternate Function mapping */
#define GPIO_AF1_TIM2          ((uint8)0x01)  /* TIM2 Alternate Function mapping */

/*
 * Description :
 * AF 2 selection.
 */
#define GPIO_AF2_TIM3          ((uint8)0x02)  /* TIM3 Alternate Function mapping */
#define GPIO_AF2_TIM4          ((uint8)0x02)  /* TIM4 Alternate Function mapping */
#define GPIO_AF2_TIM5          ((uint8)0x02)  /* TIM5 Alternate Function mapping */

/*
 * Description :
 * AF 3 selection.
 */
#define GPIO_AF3_TIM8          ((uint8)0x03)  /* TIM8 Alternate Function mapping  */
#define GPIO_AF3_TIM9          ((uint8)0x03)  /* TIM9 Alternate Function mapping  */
#define GPIO_AF3_TIM10         ((uint8)0x03)  /* TIM10 Alternate Function mapping */
#define GPIO_AF3_TIM11         ((uint8)0x03)  /* TIM11 Alternate Function mapping */

/*
 * Description :
 * AF 4 selection.
 */
#define GPIO_AF4_I2C1          ((uint8)0x04)  /* I2C1 Alternate Function mapping */
#define GPIO_AF4_I2C2          ((uint8)0x04)  /* I2C2 Alternate Function mapping */
#define GPIO_AF4_I2C3          ((uint8)0x04)  /* I2C3 Alternate Function mapping */

/*
 * Description :
 * AF 5 selection.
 */
#define GPIO_AF5_SPI1          ((uint8)0x05)  /* SPI1 Alternate Function mapping        */
#define GPIO_AF5_SPI2          ((uint8)0x05)  /* SPI2/I2S2 Alternate Function mapping   */
#define GPIO_AF5_SPI3          ((uint8)0x05)  /* SPI3/I2S3 Alternate Function mapping   */
#define GPIO_AF5_SPI4          ((uint8)0x05)  /* SPI4 Alternate Function mapping        */
#define GPIO_AF5_SPI5          ((uint8)0x05)  /* SPI5 Alternate Function mapping        */
#define GPIO_AF5_SPI6          ((uint8)0x05)  /* SPI6 Alternate Function mapping        */
#define GPIO_AF5_I2S3ext       ((uint8)0x05)  /* I2S3ext_SD Alternate Function mapping  */

/*
 * Description :
 * AF 6 selection.
 */
#define GPIO_AF6_SPI3          ((uint8)0x06)  /* SPI3/I2S3 Alternate Function mapping  */
#define GPIO_AF6_I2S2ext       ((uint8)0x06)  /* I2S2ext_SD Alternate Function mapping */
#define GPIO_AF6_SAI1          ((uint8)0x06)  /* SAI1 Alternate Function mapping       */

/*
 * Description :
 * AF 7 selection.
 */
#define GPIO_AF7_USART1        ((uint8)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8)0x07)  /* USART2 Alternate Function mapping     */
#define GPIO_AF7_USART3        ((uint8)0x07)  /* USART3 Alternate Function mapping     */
#define GPIO_AF7_I2S3ext       ((uint8)0x07)  /* I2S3ext_SD Alternate Function mapping */

/*
 * Description :
 * AF 8 selection.
 */
#define GPIO_AF8_UART4         ((uint8)0x08)  /* UART4 Alternate Function mapping  */
#define GPIO_AF8_UART5         ((uint8)0x08)  /* UART5 Alternate Function mapping  */
#define GPIO_AF8_USART6        ((uint8)0x08)  /* USART6 Alternate Function mapping */
#define GPIO_AF8_UART7         ((uint8)0x08)  /* UART7 Alternate Function mapping  */
#define GPIO_AF8_UART8         ((uint8)0x08)  /* UART8 Alternate Function mapping  */

/*
 * Description :
 * AF 9 selection.
 */
#define GPIO_AF9_CAN1          ((uint8)0x09)  /* CAN1 Alternate Function mapping    */
#define GPIO_AF9_CAN2          ((uint8)0x09)  /* CAN2 Alternate Function mapping    */
#define GPIO_AF9_TIM12         ((uint8)0x09)  /* TIM12 Alternate Function mapping   */
#define GPIO_AF9_TIM13         ((uint8)0x09)  /* TIM13 Alternate Function mapping   */
#define GPIO_AF9_TIM14         ((uint8)0x09)  /* TIM14 Alternate Function mapping   */
#define GPIO_AF9_LTDC          ((uint8)0x09)  /* LCD-TFT Alternate Function mapping */

/*
 * Description :
 * AF 10 selection.
 */
#define GPIO_AF10_OTG_FS        ((uint8)0x0A)  /* OTG_FS Alternate Function mapping */
#define GPIO_AF10_OTG_HS        ((uint8)0x0A)  /* OTG_HS Alternate Function mapping */

/*
 * Description :
 * AF 11 selection.
 */
#define GPIO_AF11_ETH           ((uint8)0x0B)  /* ETHERNET Alternate Function mapping */

/*
 * Description :
 * AF 12 selection.
 */
#define GPIO_AF12_FMC           ((uint8)0x0C)  /* FMC Alternate Function mapping                      */
#define GPIO_AF12_OTG_HS_FS     ((uint8)0x0C)  /* OTG HS configured in FS, Alternate Function mapping */
#define GPIO_AF12_SDIO          ((uint8)0x0C)  /* SDIO Alternate Function mapping                     */

/*
 * Description :
 * AF 13 selection.
 */
#define GPIO_AF13_DCMI          ((uint8)0x0D)  /* DCMI Alternate Function mapping */

/*
 * Description :
 * AF 14 selection.
 */
#define GPIO_AF14_LTDC          ((uint8)0x0E)  /* LCD-TFT Alternate Function mapping */

/*
 * Description :
 * AF 15 selection.
 */
#define GPIO_AF15_EVENTOUT      ((uint8)0x0F)  /* EVENTOUT Alternate Function mapping */



#endif /* GPIO_H_ */
