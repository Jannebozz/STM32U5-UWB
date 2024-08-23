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
//extern PCD_HandleTypeDef  hpcd_USB_OTG_FS;
extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

extern SPI_HandleTypeDef hspi2;
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
		HAL_DMA_IRQHandler(&hdma_spi2_tx);
#else
		DMA_HandleTypeDef *hdma = &hdma_spi2_tx;
		DMA_Base_Registers  *regs_dma  = (DMA_Base_Registers *)hdma->StreamBaseAddress;
		/* Clear the transfer complete flag */
		regs_dma->IFCR = DMA_FLAG_TCIF0_4 << (hdma->StreamIndex & 0x1FU);
		/* Disable the transfer complete interrupt */
		((DMA_Stream_TypeDef   *)hdma->Instance)->CR  &= ~(DMA_IT_TC);
		hdma->State = HAL_DMA_STATE_READY;
		__HAL_UNLOCK(hdma);
#endif
}

/**
 * @brief This function handles DMA1 stream2 global interrupt.
 */
void DMA_SPI_Rx_Interrupt(void)
{
	/* USER CODE BEGIN DMA1_Stream2_IRQn 0 */
#ifdef USE_ST_HAL_SPI_DRIVER
	HAL_DMA_IRQHandler(&hdma_spi2_rx);
#else
	DMA_HandleTypeDef *hdma = &hdma_spi2_rx;
	DMA_Base_Registers  *regs_dma  = (DMA_Base_Registers *)hdma->StreamBaseAddress;
	/* Clear the transfer complete flag */
	regs_dma->IFCR = DMA_FLAG_TCIF0_4 << (hdma->StreamIndex & 0x1FU);
	/* Disable the transfer complete interrupt */
	((DMA_Stream_TypeDef   *)hdma->Instance)->CR  &= ~(DMA_IT_TC);
	hdma->State = HAL_DMA_STATE_READY;
	__HAL_UNLOCK(hdma);
	uint32_t *rx_data = (uint32_t*)((DMA_Stream_TypeDef   *)hdma->Instance)->M0AR;
	//Currently don´t know how much data we actually should read, so clear whole buffer
	SCB_InvalidateDCache_by_Addr((uint32_t*)(((uint32_t)rx_data) & ~(uint32_t)0x1F), 224);
#endif

	radio1_dma_callback();

	/* USER CODE END DMA1_Stream2_IRQn 0 */
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
