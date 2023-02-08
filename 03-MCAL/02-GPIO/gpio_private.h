/********************************************************************************
 *                                                                              *
 * [FILE NAME]:   gpio_private.h                                                *
 *                                                                              *
 * [AUTHOR]:      Ahmed Saeed                                                   *
 *                                                                              *
 * [DATE]:        07/2/2023                                                     *
 *                                                                              *
 * [DESCRIPTION]: Header file for the GPIO driver Registers                     *
 *                                                                              *
 * [VERSION]:     1.0.0                                                         *
 *                                                                              *
 *******************************************************************************/

#ifndef GPIO_PRIVATE_H_
#define GPIO_PRIVATE_H_

#include "std_types.h"


/*
 * Description :
 * General Purpose I/O Registers.
 */
typedef struct
{
  volatile uint32 MODER;    /* GPIO port mode register,               Address offset: 0x00      */
  volatile uint32 OTYPER;   /* GPIO port output type register,        Address offset: 0x04      */
  volatile uint32 OSPEEDR;  /* GPIO port output speed register,       Address offset: 0x08      */
  volatile uint32 PUPDR;    /* GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  volatile uint32 IDR;      /* GPIO port input data register,         Address offset: 0x10      */
  volatile uint32 ODR;      /* GPIO port output data register,        Address offset: 0x14      */
  volatile uint32 BSRR;     /* GPIO port bit set/reset register,      Address offset: 0x18      */
  volatile uint32 LCKR;     /* GPIO port configuration lock register, Address offset: 0x1C      */
  volatile uint32 AFR[2];   /* GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;


#define PERIPH_BASE           0x40000000UL /* Peripheral base address in the alias region */

/* Peripheral memory map */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)


#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000UL)
#define GPIOJ_BASE            (AHB1PERIPH_BASE + 0x2400UL)
#define GPIOK_BASE            (AHB1PERIPH_BASE + 0x2800UL)


#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
#define GPIOI               ((GPIO_TypeDef *) GPIOI_BASE)
#define GPIOJ               ((GPIO_TypeDef *) GPIOJ_BASE)
#define GPIOK               ((GPIO_TypeDef *) GPIOK_BASE)


/******************************************************************************/
/*                                                                            */
/*                            General Purpose I/O                             */
/*                                                                            */
/******************************************************************************/
/******************  Bits definition for GPIO_MODER register  *****************/
#define GPIO_MODER_MODER0_Pos            (0U)
#define GPIO_MODER_MODER0                (0x3UL << GPIO_MODER_MODER0_Pos)       /*!< 0x00000003 */

#define GPIO_MODER_MODER1_Pos            (2U)
#define GPIO_MODER_MODER1                (0x3UL << GPIO_MODER_MODER1_Pos)       /*!< 0x0000000C */

#define GPIO_MODER_MODER2_Pos            (4U)
#define GPIO_MODER_MODER2                (0x3UL << GPIO_MODER_MODER2_Pos)       /*!< 0x00000030 */

#define GPIO_MODER_MODER3_Pos            (6U)
#define GPIO_MODER_MODER3                (0x3UL << GPIO_MODER_MODER3_Pos)       /*!< 0x000000C0 */

#define GPIO_MODER_MODER4_Pos            (8U)
#define GPIO_MODER_MODER4                (0x3UL << GPIO_MODER_MODER4_Pos)       /*!< 0x00000300 */

#define GPIO_MODER_MODER5_Pos            (10U)
#define GPIO_MODER_MODER5                (0x3UL << GPIO_MODER_MODER5_Pos)       /*!< 0x00000C00 */

#define GPIO_MODER_MODER6_Pos            (12U)
#define GPIO_MODER_MODER6                (0x3UL << GPIO_MODER_MODER6_Pos)       /*!< 0x00003000 */

#define GPIO_MODER_MODER7_Pos            (14U)
#define GPIO_MODER_MODER7                (0x3UL << GPIO_MODER_MODER7_Pos)       /*!< 0x0000C000 */

#define GPIO_MODER_MODER8_Pos            (16U)
#define GPIO_MODER_MODER8                (0x3UL << GPIO_MODER_MODER8_Pos)       /*!< 0x00030000 */

#define GPIO_MODER_MODER9_Pos            (18U)
#define GPIO_MODER_MODER9                (0x3UL << GPIO_MODER_MODER9_Pos)       /*!< 0x000C0000 */

#define GPIO_MODER_MODER10_Pos           (20U)
#define GPIO_MODER_MODER10               (0x3UL << GPIO_MODER_MODER10_Pos)      /*!< 0x00300000 */

#define GPIO_MODER_MODER11_Pos           (22U)
#define GPIO_MODER_MODER11               (0x3UL << GPIO_MODER_MODER11_Pos)      /*!< 0x00C00000 */

#define GPIO_MODER_MODER12_Pos           (24U)
#define GPIO_MODER_MODER12               (0x3UL << GPIO_MODER_MODER12_Pos)      /*!< 0x03000000 */

#define GPIO_MODER_MODER13_Pos           (26U)
#define GPIO_MODER_MODER13               (0x3UL << GPIO_MODER_MODER13_Pos)      /*!< 0x0C000000 */

#define GPIO_MODER_MODER14_Pos           (28U)
#define GPIO_MODER_MODER14               (0x3UL << GPIO_MODER_MODER14_Pos)      /*!< 0x30000000 */

#define GPIO_MODER_MODER15_Pos           (30U)
#define GPIO_MODER_MODER15               (0x3UL << GPIO_MODER_MODER15_Pos)      /*!< 0xC0000000 */

/******************  Bits definition for GPIO_OTYPER register  ****************/
#define GPIO_OTYPER_OT0_Pos              (0U)
#define GPIO_OTYPER_OT0                  (0x1UL << GPIO_OTYPER_OT0_Pos)         /*!< 0x00000001 */

#define GPIO_OTYPER_OT1_Pos              (1U)
#define GPIO_OTYPER_OT1                  (0x1UL << GPIO_OTYPER_OT1_Pos)         /*!< 0x00000002 */

#define GPIO_OTYPER_OT2_Pos              (2U)
#define GPIO_OTYPER_OT2                  (0x1UL << GPIO_OTYPER_OT2_Pos)         /*!< 0x00000004 */

#define GPIO_OTYPER_OT3_Pos              (3U)
#define GPIO_OTYPER_OT3                  (0x1UL << GPIO_OTYPER_OT3_Pos)         /*!< 0x00000008 */

#define GPIO_OTYPER_OT4_Pos              (4U)
#define GPIO_OTYPER_OT4                  (0x1UL << GPIO_OTYPER_OT4_Pos)         /*!< 0x00000010 */

#define GPIO_OTYPER_OT5_Pos              (5U)
#define GPIO_OTYPER_OT5                  (0x1UL << GPIO_OTYPER_OT5_Pos)         /*!< 0x00000020 */

#define GPIO_OTYPER_OT6_Pos              (6U)
#define GPIO_OTYPER_OT6                  (0x1UL << GPIO_OTYPER_OT6_Pos)         /*!< 0x00000040 */

#define GPIO_OTYPER_OT7_Pos              (7U)
#define GPIO_OTYPER_OT7                  (0x1UL << GPIO_OTYPER_OT7_Pos)         /*!< 0x00000080 */

#define GPIO_OTYPER_OT8_Pos              (8U)
#define GPIO_OTYPER_OT8                  (0x1UL << GPIO_OTYPER_OT8_Pos)         /*!< 0x00000100 */

#define GPIO_OTYPER_OT9_Pos              (9U)
#define GPIO_OTYPER_OT9                  (0x1UL << GPIO_OTYPER_OT9_Pos)         /*!< 0x00000200 */

#define GPIO_OTYPER_OT10_Pos             (10U)
#define GPIO_OTYPER_OT10                 (0x1UL << GPIO_OTYPER_OT10_Pos)        /*!< 0x00000400 */

#define GPIO_OTYPER_OT11_Pos             (11U)
#define GPIO_OTYPER_OT11                 (0x1UL << GPIO_OTYPER_OT11_Pos)        /*!< 0x00000800 */

#define GPIO_OTYPER_OT12_Pos             (12U)
#define GPIO_OTYPER_OT12                 (0x1UL << GPIO_OTYPER_OT12_Pos)        /*!< 0x00001000 */

#define GPIO_OTYPER_OT13_Pos             (13U)
#define GPIO_OTYPER_OT13                 (0x1UL << GPIO_OTYPER_OT13_Pos)        /*!< 0x00002000 */

#define GPIO_OTYPER_OT14_Pos             (14U)
#define GPIO_OTYPER_OT14                 (0x1UL << GPIO_OTYPER_OT14_Pos)        /*!< 0x00004000 */

#define GPIO_OTYPER_OT15_Pos             (15U)
#define GPIO_OTYPER_OT15                 (0x1UL << GPIO_OTYPER_OT15_Pos)        /*!< 0x00008000 */

/******************  Bits definition for GPIO_OSPEEDR register  ***************/
#define GPIO_OSPEEDR_OSPEED0_Pos         (0U)
#define GPIO_OSPEEDR_OSPEED0             (0x3UL << GPIO_OSPEEDR_OSPEED0_Pos)    /*!< 0x00000003 */

#define GPIO_OSPEEDR_OSPEED1_Pos         (2U)
#define GPIO_OSPEEDR_OSPEED1             (0x3UL << GPIO_OSPEEDR_OSPEED1_Pos)    /*!< 0x0000000C */

#define GPIO_OSPEEDR_OSPEED2_Pos         (4U)
#define GPIO_OSPEEDR_OSPEED2             (0x3UL << GPIO_OSPEEDR_OSPEED2_Pos)    /*!< 0x00000030 */

#define GPIO_OSPEEDR_OSPEED3_Pos         (6U)
#define GPIO_OSPEEDR_OSPEED3             (0x3UL << GPIO_OSPEEDR_OSPEED3_Pos)    /*!< 0x000000C0 */

#define GPIO_OSPEEDR_OSPEED4_Pos         (8U)
#define GPIO_OSPEEDR_OSPEED4             (0x3UL << GPIO_OSPEEDR_OSPEED4_Pos)    /*!< 0x00000300 */

#define GPIO_OSPEEDR_OSPEED5_Pos         (10U)
#define GPIO_OSPEEDR_OSPEED5             (0x3UL << GPIO_OSPEEDR_OSPEED5_Pos)    /*!< 0x00000C00 */

#define GPIO_OSPEEDR_OSPEED6_Pos         (12U)
#define GPIO_OSPEEDR_OSPEED6             (0x3UL << GPIO_OSPEEDR_OSPEED6_Pos)    /*!< 0x00003000 */

#define GPIO_OSPEEDR_OSPEED7_Pos         (14U)
#define GPIO_OSPEEDR_OSPEED7             (0x3UL << GPIO_OSPEEDR_OSPEED7_Pos)    /*!< 0x0000C000 */

#define GPIO_OSPEEDR_OSPEED8_Pos         (16U)
#define GPIO_OSPEEDR_OSPEED8             (0x3UL << GPIO_OSPEEDR_OSPEED8_Pos)    /*!< 0x00030000 */

#define GPIO_OSPEEDR_OSPEED9_Pos         (18U)
#define GPIO_OSPEEDR_OSPEED9             (0x3UL << GPIO_OSPEEDR_OSPEED9_Pos)    /*!< 0x000C0000 */

#define GPIO_OSPEEDR_OSPEED10_Pos        (20U)
#define GPIO_OSPEEDR_OSPEED10            (0x3UL << GPIO_OSPEEDR_OSPEED10_Pos)   /*!< 0x00300000 */

#define GPIO_OSPEEDR_OSPEED11_Pos        (22U)
#define GPIO_OSPEEDR_OSPEED11            (0x3UL << GPIO_OSPEEDR_OSPEED11_Pos)   /*!< 0x00C00000 */

#define GPIO_OSPEEDR_OSPEED12_Pos        (24U)
#define GPIO_OSPEEDR_OSPEED12            (0x3UL << GPIO_OSPEEDR_OSPEED12_Pos)   /*!< 0x03000000 */

#define GPIO_OSPEEDR_OSPEED13_Pos        (26U)
#define GPIO_OSPEEDR_OSPEED13            (0x3UL << GPIO_OSPEEDR_OSPEED13_Pos)   /*!< 0x0C000000 */

#define GPIO_OSPEEDR_OSPEED14_Pos        (28U)
#define GPIO_OSPEEDR_OSPEED14            (0x3UL << GPIO_OSPEEDR_OSPEED14_Pos)   /*!< 0x30000000 */

#define GPIO_OSPEEDR_OSPEED15_Pos        (30U)
#define GPIO_OSPEEDR_OSPEED15            (0x3UL << GPIO_OSPEEDR_OSPEED15_Pos)   /*!< 0xC0000000 */

/******************  Bits definition for GPIO_PUPDR register  *****************/
#define GPIO_PUPDR_PUPD0_Pos             (0U)
#define GPIO_PUPDR_PUPD0                 (0x3UL << GPIO_PUPDR_PUPD0_Pos)        /*!< 0x00000003 */

#define GPIO_PUPDR_PUPD1_Pos             (2U)
#define GPIO_PUPDR_PUPD1                 (0x3UL << GPIO_PUPDR_PUPD1_Pos)        /*!< 0x0000000C */

#define GPIO_PUPDR_PUPD2_Pos             (4U)
#define GPIO_PUPDR_PUPD2                 (0x3UL << GPIO_PUPDR_PUPD2_Pos)        /*!< 0x00000030 */

#define GPIO_PUPDR_PUPD3_Pos             (6U)
#define GPIO_PUPDR_PUPD3                 (0x3UL << GPIO_PUPDR_PUPD3_Pos)        /*!< 0x000000C0 */

#define GPIO_PUPDR_PUPD4_Pos             (8U)
#define GPIO_PUPDR_PUPD4                 (0x3UL << GPIO_PUPDR_PUPD4_Pos)        /*!< 0x00000300 */

#define GPIO_PUPDR_PUPD5_Pos             (10U)
#define GPIO_PUPDR_PUPD5                 (0x3UL << GPIO_PUPDR_PUPD5_Pos)        /*!< 0x00000C00 */

#define GPIO_PUPDR_PUPD6_Pos             (12U)
#define GPIO_PUPDR_PUPD6                 (0x3UL << GPIO_PUPDR_PUPD6_Pos)        /*!< 0x00003000 */

#define GPIO_PUPDR_PUPD7_Pos             (14U)
#define GPIO_PUPDR_PUPD7                 (0x3UL << GPIO_PUPDR_PUPD7_Pos)        /*!< 0x0000C000 */

#define GPIO_PUPDR_PUPD8_Pos             (16U)
#define GPIO_PUPDR_PUPD8                 (0x3UL << GPIO_PUPDR_PUPD8_Pos)        /*!< 0x00030000 */

#define GPIO_PUPDR_PUPD9_Pos             (18U)
#define GPIO_PUPDR_PUPD9                 (0x3UL << GPIO_PUPDR_PUPD9_Pos)        /*!< 0x000C0000 */

#define GPIO_PUPDR_PUPD10_Pos            (20U)
#define GPIO_PUPDR_PUPD10                (0x3UL << GPIO_PUPDR_PUPD10_Pos)       /*!< 0x00300000 */

#define GPIO_PUPDR_PUPD11_Pos            (22U)
#define GPIO_PUPDR_PUPD11                (0x3UL << GPIO_PUPDR_PUPD11_Pos)       /*!< 0x00C00000 */

#define GPIO_PUPDR_PUPD12_Pos            (24U)
#define GPIO_PUPDR_PUPD12                (0x3UL << GPIO_PUPDR_PUPD12_Pos)       /*!< 0x03000000 */

#define GPIO_PUPDR_PUPD13_Pos            (26U)
#define GPIO_PUPDR_PUPD13                (0x3UL << GPIO_PUPDR_PUPD13_Pos)       /*!< 0x0C000000 */

#define GPIO_PUPDR_PUPD14_Pos            (28U)
#define GPIO_PUPDR_PUPD14                (0x3UL << GPIO_PUPDR_PUPD14_Pos)       /*!< 0x30000000 */

#define GPIO_PUPDR_PUPD15_Pos            (30U)
#define GPIO_PUPDR_PUPD15                (0x3UL << GPIO_PUPDR_PUPD15_Pos)       /*!< 0xC0000000 */

/******************  Bits definition for GPIO_IDR register  *******************/
#define GPIO_IDR_ID0_Pos                 (0U)
#define GPIO_IDR_ID0                     (0x1UL << GPIO_IDR_ID0_Pos)            /*!< 0x00000001 */

#define GPIO_IDR_ID1_Pos                 (1U)
#define GPIO_IDR_ID1                     (0x1UL << GPIO_IDR_ID1_Pos)            /*!< 0x00000002 */

#define GPIO_IDR_ID2_Pos                 (2U)
#define GPIO_IDR_ID2                     (0x1UL << GPIO_IDR_ID2_Pos)            /*!< 0x00000004 */

#define GPIO_IDR_ID3_Pos                 (3U)
#define GPIO_IDR_ID3                     (0x1UL << GPIO_IDR_ID3_Pos)            /*!< 0x00000008 */

#define GPIO_IDR_ID4_Pos                 (4U)
#define GPIO_IDR_ID4                     (0x1UL << GPIO_IDR_ID4_Pos)            /*!< 0x00000010 */

#define GPIO_IDR_ID5_Pos                 (5U)
#define GPIO_IDR_ID5                     (0x1UL << GPIO_IDR_ID5_Pos)            /*!< 0x00000020 */

#define GPIO_IDR_ID6_Pos                 (6U)
#define GPIO_IDR_ID6                     (0x1UL << GPIO_IDR_ID6_Pos)            /*!< 0x00000040 */

#define GPIO_IDR_ID7_Pos                 (7U)
#define GPIO_IDR_ID7                     (0x1UL << GPIO_IDR_ID7_Pos)            /*!< 0x00000080 */

#define GPIO_IDR_ID8_Pos                 (8U)
#define GPIO_IDR_ID8                     (0x1UL << GPIO_IDR_ID8_Pos)            /*!< 0x00000100 */

#define GPIO_IDR_ID9_Pos                 (9U)
#define GPIO_IDR_ID9                     (0x1UL << GPIO_IDR_ID9_Pos)            /*!< 0x00000200 */

#define GPIO_IDR_ID10_Pos                (10U)
#define GPIO_IDR_ID10                    (0x1UL << GPIO_IDR_ID10_Pos)           /*!< 0x00000400 */

#define GPIO_IDR_ID11_Pos                (11U)
#define GPIO_IDR_ID11                    (0x1UL << GPIO_IDR_ID11_Pos)           /*!< 0x00000800 */

#define GPIO_IDR_ID12_Pos                (12U)
#define GPIO_IDR_ID12                    (0x1UL << GPIO_IDR_ID12_Pos)           /*!< 0x00001000 */

#define GPIO_IDR_ID13_Pos                (13U)
#define GPIO_IDR_ID13                    (0x1UL << GPIO_IDR_ID13_Pos)           /*!< 0x00002000 */

#define GPIO_IDR_ID14_Pos                (14U)
#define GPIO_IDR_ID14                    (0x1UL << GPIO_IDR_ID14_Pos)           /*!< 0x00004000 */

#define GPIO_IDR_ID15_Pos                (15U)
#define GPIO_IDR_ID15                    (0x1UL << GPIO_IDR_ID15_Pos)           /*!< 0x00008000 */

/******************  Bits definition for GPIO_ODR register  *******************/
#define GPIO_ODR_OD0_Pos                 (0U)
#define GPIO_ODR_OD0                     (0x1UL << GPIO_ODR_OD0_Pos)            /*!< 0x00000001 */

#define GPIO_ODR_OD1_Pos                 (1U)
#define GPIO_ODR_OD1                     (0x1UL << GPIO_ODR_OD1_Pos)            /*!< 0x00000002 */

#define GPIO_ODR_OD2_Pos                 (2U)
#define GPIO_ODR_OD2                     (0x1UL << GPIO_ODR_OD2_Pos)            /*!< 0x00000004 */

#define GPIO_ODR_OD3_Pos                 (3U)
#define GPIO_ODR_OD3                     (0x1UL << GPIO_ODR_OD3_Pos)            /*!< 0x00000008 */

#define GPIO_ODR_OD4_Pos                 (4U)
#define GPIO_ODR_OD4                     (0x1UL << GPIO_ODR_OD4_Pos)            /*!< 0x00000010 */

#define GPIO_ODR_OD5_Pos                 (5U)
#define GPIO_ODR_OD5                     (0x1UL << GPIO_ODR_OD5_Pos)            /*!< 0x00000020 */

#define GPIO_ODR_OD6_Pos                 (6U)
#define GPIO_ODR_OD6                     (0x1UL << GPIO_ODR_OD6_Pos)            /*!< 0x00000040 */

#define GPIO_ODR_OD7_Pos                 (7U)
#define GPIO_ODR_OD7                     (0x1UL << GPIO_ODR_OD7_Pos)            /*!< 0x00000080 */

#define GPIO_ODR_OD8_Pos                 (8U)
#define GPIO_ODR_OD8                     (0x1UL << GPIO_ODR_OD8_Pos)            /*!< 0x00000100 */

#define GPIO_ODR_OD9_Pos                 (9U)
#define GPIO_ODR_OD9                     (0x1UL << GPIO_ODR_OD9_Pos)            /*!< 0x00000200 */

#define GPIO_ODR_OD10_Pos                (10U)
#define GPIO_ODR_OD10                    (0x1UL << GPIO_ODR_OD10_Pos)           /*!< 0x00000400 */

#define GPIO_ODR_OD11_Pos                (11U)
#define GPIO_ODR_OD11                    (0x1UL << GPIO_ODR_OD11_Pos)           /*!< 0x00000800 */

#define GPIO_ODR_OD12_Pos                (12U)
#define GPIO_ODR_OD12                    (0x1UL << GPIO_ODR_OD12_Pos)           /*!< 0x00001000 */

#define GPIO_ODR_OD13_Pos                (13U)
#define GPIO_ODR_OD13                    (0x1UL << GPIO_ODR_OD13_Pos)           /*!< 0x00002000 */

#define GPIO_ODR_OD14_Pos                (14U)
#define GPIO_ODR_OD14                    (0x1UL << GPIO_ODR_OD14_Pos)           /*!< 0x00004000 */

#define GPIO_ODR_OD15_Pos                (15U)
#define GPIO_ODR_OD15                    (0x1UL << GPIO_ODR_OD15_Pos)           /*!< 0x00008000 */

/******************  Bits definition for GPIO_BSRR register  ******************/
#define GPIO_BSRR_BS0_Pos                (0U)
#define GPIO_BSRR_BS0                    (0x1UL << GPIO_BSRR_BS0_Pos)           /*!< 0x00000001 */

#define GPIO_BSRR_BS1_Pos                (1U)
#define GPIO_BSRR_BS1                    (0x1UL << GPIO_BSRR_BS1_Pos)           /*!< 0x00000002 */

#define GPIO_BSRR_BS2_Pos                (2U)
#define GPIO_BSRR_BS2                    (0x1UL << GPIO_BSRR_BS2_Pos)           /*!< 0x00000004 */

#define GPIO_BSRR_BS3_Pos                (3U)
#define GPIO_BSRR_BS3                    (0x1UL << GPIO_BSRR_BS3_Pos)           /*!< 0x00000008 */

#define GPIO_BSRR_BS4_Pos                (4U)
#define GPIO_BSRR_BS4                    (0x1UL << GPIO_BSRR_BS4_Pos)           /*!< 0x00000010 */

#define GPIO_BSRR_BS5_Pos                (5U)
#define GPIO_BSRR_BS5                    (0x1UL << GPIO_BSRR_BS5_Pos)           /*!< 0x00000020 */

#define GPIO_BSRR_BS6_Pos                (6U)
#define GPIO_BSRR_BS6                    (0x1UL << GPIO_BSRR_BS6_Pos)           /*!< 0x00000040 */

#define GPIO_BSRR_BS7_Pos                (7U)
#define GPIO_BSRR_BS7                    (0x1UL << GPIO_BSRR_BS7_Pos)           /*!< 0x00000080 */

#define GPIO_BSRR_BS8_Pos                (8U)
#define GPIO_BSRR_BS8                    (0x1UL << GPIO_BSRR_BS8_Pos)           /*!< 0x00000100 */

#define GPIO_BSRR_BS9_Pos                (9U)
#define GPIO_BSRR_BS9                    (0x1UL << GPIO_BSRR_BS9_Pos)           /*!< 0x00000200 */

#define GPIO_BSRR_BS10_Pos               (10U)
#define GPIO_BSRR_BS10                   (0x1UL << GPIO_BSRR_BS10_Pos)          /*!< 0x00000400 */

#define GPIO_BSRR_BS11_Pos               (11U)
#define GPIO_BSRR_BS11                   (0x1UL << GPIO_BSRR_BS11_Pos)          /*!< 0x00000800 */

#define GPIO_BSRR_BS12_Pos               (12U)
#define GPIO_BSRR_BS12                   (0x1UL << GPIO_BSRR_BS12_Pos)          /*!< 0x00001000 */

#define GPIO_BSRR_BS13_Pos               (13U)
#define GPIO_BSRR_BS13                   (0x1UL << GPIO_BSRR_BS13_Pos)          /*!< 0x00002000 */

#define GPIO_BSRR_BS14_Pos               (14U)
#define GPIO_BSRR_BS14                   (0x1UL << GPIO_BSRR_BS14_Pos)          /*!< 0x00004000 */

#define GPIO_BSRR_BS15_Pos               (15U)
#define GPIO_BSRR_BS15                   (0x1UL << GPIO_BSRR_BS15_Pos)          /*!< 0x00008000 */

#define GPIO_BSRR_BR0_Pos                (16U)
#define GPIO_BSRR_BR0                    (0x1UL << GPIO_BSRR_BR0_Pos)           /*!< 0x00010000 */

#define GPIO_BSRR_BR1_Pos                (17U)
#define GPIO_BSRR_BR1                    (0x1UL << GPIO_BSRR_BR1_Pos)           /*!< 0x00020000 */

#define GPIO_BSRR_BR2_Pos                (18U)
#define GPIO_BSRR_BR2                    (0x1UL << GPIO_BSRR_BR2_Pos)           /*!< 0x00040000 */

#define GPIO_BSRR_BR3_Pos                (19U)
#define GPIO_BSRR_BR3                    (0x1UL << GPIO_BSRR_BR3_Pos)           /*!< 0x00080000 */

#define GPIO_BSRR_BR4_Pos                (20U)
#define GPIO_BSRR_BR4                    (0x1UL << GPIO_BSRR_BR4_Pos)           /*!< 0x00100000 */

#define GPIO_BSRR_BR5_Pos                (21U)
#define GPIO_BSRR_BR5                    (0x1UL << GPIO_BSRR_BR5_Pos)           /*!< 0x00200000 */

#define GPIO_BSRR_BR6_Pos                (22U)
#define GPIO_BSRR_BR6                    (0x1UL << GPIO_BSRR_BR6_Pos)           /*!< 0x00400000 */

#define GPIO_BSRR_BR7_Pos                (23U)
#define GPIO_BSRR_BR7                    (0x1UL << GPIO_BSRR_BR7_Pos)           /*!< 0x00800000 */

#define GPIO_BSRR_BR8_Pos                (24U)
#define GPIO_BSRR_BR8                    (0x1UL << GPIO_BSRR_BR8_Pos)           /*!< 0x01000000 */

#define GPIO_BSRR_BR9_Pos                (25U)
#define GPIO_BSRR_BR9                    (0x1UL << GPIO_BSRR_BR9_Pos)           /*!< 0x02000000 */

#define GPIO_BSRR_BR10_Pos               (26U)
#define GPIO_BSRR_BR10                   (0x1UL << GPIO_BSRR_BR10_Pos)          /*!< 0x04000000 */

#define GPIO_BSRR_BR11_Pos               (27U)
#define GPIO_BSRR_BR11                   (0x1UL << GPIO_BSRR_BR11_Pos)          /*!< 0x08000000 */

#define GPIO_BSRR_BR12_Pos               (28U)
#define GPIO_BSRR_BR12                   (0x1UL << GPIO_BSRR_BR12_Pos)          /*!< 0x10000000 */

#define GPIO_BSRR_BR13_Pos               (29U)
#define GPIO_BSRR_BR13                   (0x1UL << GPIO_BSRR_BR13_Pos)          /*!< 0x20000000 */

#define GPIO_BSRR_BR14_Pos               (30U)
#define GPIO_BSRR_BR14                   (0x1UL << GPIO_BSRR_BR14_Pos)          /*!< 0x40000000 */

#define GPIO_BSRR_BR15_Pos               (31U)
#define GPIO_BSRR_BR15                   (0x1UL << GPIO_BSRR_BR15_Pos)          /*!< 0x80000000 */

/****************** Bit definition for GPIO_LCKR register *********************/
#define GPIO_LCKR_LCK0_Pos               (0U)
#define GPIO_LCKR_LCK0                   (0x1UL << GPIO_LCKR_LCK0_Pos)          /*!< 0x00000001 */

#define GPIO_LCKR_LCK1_Pos               (1U)
#define GPIO_LCKR_LCK1                   (0x1UL << GPIO_LCKR_LCK1_Pos)          /*!< 0x00000002 */

#define GPIO_LCKR_LCK2_Pos               (2U)
#define GPIO_LCKR_LCK2                   (0x1UL << GPIO_LCKR_LCK2_Pos)          /*!< 0x00000004 */

#define GPIO_LCKR_LCK3_Pos               (3U)
#define GPIO_LCKR_LCK3                   (0x1UL << GPIO_LCKR_LCK3_Pos)          /*!< 0x00000008 */

#define GPIO_LCKR_LCK4_Pos               (4U)
#define GPIO_LCKR_LCK4                   (0x1UL << GPIO_LCKR_LCK4_Pos)          /*!< 0x00000010 */

#define GPIO_LCKR_LCK5_Pos               (5U)
#define GPIO_LCKR_LCK5                   (0x1UL << GPIO_LCKR_LCK5_Pos)          /*!< 0x00000020 */

#define GPIO_LCKR_LCK6_Pos               (6U)
#define GPIO_LCKR_LCK6                   (0x1UL << GPIO_LCKR_LCK6_Pos)          /*!< 0x00000040 */

#define GPIO_LCKR_LCK7_Pos               (7U)
#define GPIO_LCKR_LCK7                   (0x1UL << GPIO_LCKR_LCK7_Pos)          /*!< 0x00000080 */

#define GPIO_LCKR_LCK8_Pos               (8U)
#define GPIO_LCKR_LCK8                   (0x1UL << GPIO_LCKR_LCK8_Pos)          /*!< 0x00000100 */

#define GPIO_LCKR_LCK9_Pos               (9U)
#define GPIO_LCKR_LCK9                   (0x1UL << GPIO_LCKR_LCK9_Pos)          /*!< 0x00000200 */

#define GPIO_LCKR_LCK10_Pos              (10U)
#define GPIO_LCKR_LCK10                  (0x1UL << GPIO_LCKR_LCK10_Pos)         /*!< 0x00000400 */

#define GPIO_LCKR_LCK11_Pos              (11U)
#define GPIO_LCKR_LCK11                  (0x1UL << GPIO_LCKR_LCK11_Pos)         /*!< 0x00000800 */

#define GPIO_LCKR_LCK12_Pos              (12U)
#define GPIO_LCKR_LCK12                  (0x1UL << GPIO_LCKR_LCK12_Pos)         /*!< 0x00001000 */

#define GPIO_LCKR_LCK13_Pos              (13U)
#define GPIO_LCKR_LCK13                  (0x1UL << GPIO_LCKR_LCK13_Pos)         /*!< 0x00002000 */

#define GPIO_LCKR_LCK14_Pos              (14U)
#define GPIO_LCKR_LCK14                  (0x1UL << GPIO_LCKR_LCK14_Pos)         /*!< 0x00004000 */

#define GPIO_LCKR_LCK15_Pos              (15U)
#define GPIO_LCKR_LCK15                  (0x1UL << GPIO_LCKR_LCK15_Pos)         /*!< 0x00008000 */

#define GPIO_LCKR_LCKK_Pos               (16U)
#define GPIO_LCKR_LCKK                   (0x1UL << GPIO_LCKR_LCKK_Pos)          /*!< 0x00010000 */

/****************** Bit definition for GPIO_AFRL register *********************/
#define GPIO_AFRL_AFSEL0_Pos             (0U)
#define GPIO_AFRL_AFSEL0                 (0xFUL << GPIO_AFRL_AFSEL0_Pos)        /*!< 0x0000000F */

#define GPIO_AFRL_AFSEL1_Pos             (4U)
#define GPIO_AFRL_AFSEL1                 (0xFUL << GPIO_AFRL_AFSEL1_Pos)        /*!< 0x000000F0 */

#define GPIO_AFRL_AFSEL2_Pos             (8U)
#define GPIO_AFRL_AFSEL2                 (0xFUL << GPIO_AFRL_AFSEL2_Pos)        /*!< 0x00000F00 */

#define GPIO_AFRL_AFSEL3_Pos             (12U)
#define GPIO_AFRL_AFSEL3                 (0xFUL << GPIO_AFRL_AFSEL3_Pos)        /*!< 0x0000F000 */

#define GPIO_AFRL_AFSEL4_Pos             (16U)
#define GPIO_AFRL_AFSEL4                 (0xFUL << GPIO_AFRL_AFSEL4_Pos)        /*!< 0x000F0000 */

#define GPIO_AFRL_AFSEL5_Pos             (20U)
#define GPIO_AFRL_AFSEL5                 (0xFUL << GPIO_AFRL_AFSEL5_Pos)        /*!< 0x00F00000 */

#define GPIO_AFRL_AFSEL6_Pos             (24U)
#define GPIO_AFRL_AFSEL6                 (0xFUL << GPIO_AFRL_AFSEL6_Pos)        /*!< 0x0F000000 */

#define GPIO_AFRL_AFSEL7_Pos             (28U)
#define GPIO_AFRL_AFSEL7                 (0xFUL << GPIO_AFRL_AFSEL7_Pos)        /*!< 0xF0000000 */

/****************** Bit definition for GPIO_AFRH register *********************/
#define GPIO_AFRH_AFSEL8_Pos             (0U)
#define GPIO_AFRH_AFSEL8                 (0xFUL << GPIO_AFRH_AFSEL8_Pos)        /*!< 0x0000000F */

#define GPIO_AFRH_AFSEL9_Pos             (4U)
#define GPIO_AFRH_AFSEL9                 (0xFUL << GPIO_AFRH_AFSEL9_Pos)        /*!< 0x000000F0 */

#define GPIO_AFRH_AFSEL10_Pos            (8U)
#define GPIO_AFRH_AFSEL10                (0xFUL << GPIO_AFRH_AFSEL10_Pos)       /*!< 0x00000F00 */

#define GPIO_AFRH_AFSEL11_Pos            (12U)
#define GPIO_AFRH_AFSEL11                (0xFUL << GPIO_AFRH_AFSEL11_Pos)       /*!< 0x0000F000 */

#define GPIO_AFRH_AFSEL12_Pos            (16U)
#define GPIO_AFRH_AFSEL12                (0xFUL << GPIO_AFRH_AFSEL12_Pos)       /*!< 0x000F0000 */

#define GPIO_AFRH_AFSEL13_Pos            (20U)
#define GPIO_AFRH_AFSEL13                (0xFUL << GPIO_AFRH_AFSEL13_Pos)       /*!< 0x00F00000 */

#define GPIO_AFRH_AFSEL14_Pos            (24U)
#define GPIO_AFRH_AFSEL14                (0xFUL << GPIO_AFRH_AFSEL14_Pos)       /*!< 0x0F000000 */

#define GPIO_AFRH_AFSEL15_Pos            (28U)
#define GPIO_AFRH_AFSEL15                (0xFUL << GPIO_AFRH_AFSEL15_Pos)       /*!< 0xF0000000 */


/*
 * Description :
 * External Interrupt/Event Controller.
 */
typedef struct
{
  volatile uint32 IMR;    /* EXTI Interrupt mask register,            Address offset: 0x00 */
  volatile uint32 EMR;    /* EXTI Event mask register,                Address offset: 0x04 */
  volatile uint32 RTSR;   /* EXTI Rising trigger selection register,  Address offset: 0x08 */
  volatile uint32 FTSR;   /* EXTI Falling trigger selection register, Address offset: 0x0C */
  volatile uint32 SWIER;  /* EXTI Software interrupt event register,  Address offset: 0x10 */
  volatile uint32 PR;     /* EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define EXTI                  ((EXTI_TypeDef *) EXTI_BASE)


/*
 * Description :
 * System configuration controller.
 */
typedef struct
{
  volatile uint32 MEMRMP;       /* SYSCFG memory remap register,                      Address offset: 0x00      */
  volatile uint32 PMC;          /* SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  volatile uint32 EXTICR[4];    /* SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32      RESERVED[2];      /* Reserved, 0x18-0x1C                                                          */
  volatile uint32 CMPCR;        /* SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;

#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define SYSCFG                ((SYSCFG_TypeDef *) SYSCFG_BASE)


#endif /* GPIO_PRIVATE_H_ */
