/** @file  myevk_it.c
 *  @brief This module controls interrupt related features.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* Includes ------------------------------------------------------------------*/
#include "myevk_it.h"

/* EXTERNS ********************************************************************/
extern SPI_HandleTypeDef hspi1;
/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static void default_irq_callback(void);
/* PRIVATE GLOBALS ************************************************************/
static uint32_t nested_critical;
static irq_callback exti1_irq_callback       = default_irq_callback;
static irq_callback radio1_dma_callback      = default_irq_callback;
static irq_callback pendsv_irq_callback      = default_irq_callback;

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


/* PUBLIC FUNCTION ***********************************************************/
void myevk_set_radio_irq_callback(irq_callback callback)
{
    exti1_irq_callback = callback;
}

void myevk_set_radio_dma_rx_callback(irq_callback callback)
{
    radio1_dma_callback = callback;
}

void myevk_set_pendsv_callback(irq_callback callback)
{
    pendsv_irq_callback = callback;
}

void PendSV_Interrupt_Handler(void)
{
	/* USER CODE BEGIN PendSV_IRQn 0 */
	CLEAR_BIT(SCB->ICSR, SCB_ICSR_PENDSVSET_Msk);
	pendsv_irq_callback();
	/* USER CODE END PendSV_IRQn 0 */
}

void DMA_SPI_Tx_Interrupt(void)
{
	/* USER CODE BEGIN DMA1_Stream1_IRQn 0 */
#ifdef USE_ST_HAL_SPI_DRIVER
		HAL_DMA_IRQHandler((&hspi1)->hdmatx);
#else
	#error "This is not implemented for STM32U5 yet"
#endif
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA_SPI_Rx_Interrupt(void)
{
	/* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
#ifdef USE_ST_HAL_SPI_DRIVER
	HAL_DMA_IRQHandler((&hspi1)->hdmarx);
#else
#error "This is not implemented for STM32U5 yet"
#endif
	radio1_dma_callback();
}

void Radio_IRQ_Handler(void)
{
	/* EXTI line interrupt detected */
	//HAL_GPIO_EXTI_IRQHandler(RADIO_IRQ_PIN);
	exti1_irq_callback();
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Default interrupt used when initializing callbacks.
 */
static void default_irq_callback(void)
{
    return;
}
