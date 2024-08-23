/** @file  myevk_radio.c
 *  @brief This module controls the peripherals for the SR10x0 radio.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "myevk_it.h"
#include "myevk_radio.h"
#include "uwb_task.h"
#include "spi.h"
#include "jpeg_lcd.h"
#include "../../../Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
/* PRIVATE GLOBALS ************************************************************/

DMA_HandleTypeDef  hradio_dma_spi_rx;
DMA_HandleTypeDef  hradio_dma_spi_tx;

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;


/* PUBLIC FUNCTIONS ***********************************************************/
bool myevk_radio_read_irq_pin(void)
{
    if (HAL_GPIO_ReadPin(SR1020_INT_GPIO_Port, SR1020_INT_Pin)) {
        return true;
    } else {
        return false;
    }
}

void myevk_radio_enable_irq_it(void)
{
	SET_BIT(EXTI_D1->IMR1, SR1020_INT_Pin);
}

void myevk_radio_disable_irq_it(void)
{
	CLEAR_BIT(EXTI_D1->IMR1, SR1020_INT_Pin);
	SET_BIT(EXTI_D1->PR1, SR1020_INT_Pin);
}
void myevk_radio_enable_dma_irq_it(void)
{
    NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

void myevk_radio_disable_dma_irq_it(void)
{
    NVIC_DisableIRQ(DMA1_Stream2_IRQn);
}

void myevk_radio_set_shutdown_pin(void)
{
   HAL_GPIO_WritePin(SR1020_SHDWN_GPIO_Port, SR1020_SHDWN_Pin, GPIO_PIN_SET);
}

void myevk_radio_reset_shutdown_pin(void)
{
    HAL_GPIO_WritePin(SR1020_SHDWN_GPIO_Port, SR1020_SHDWN_Pin, GPIO_PIN_RESET);
}

void myevk_radio_set_reset_pin(void)
{
    HAL_GPIO_WritePin(SR1020_RESET_GPIO_Port, SR1020_RESET_Pin, GPIO_PIN_SET);
}

void myevk_radio_reset_reset_pin(void)
{
    HAL_GPIO_WritePin(SR1020_RESET_GPIO_Port, SR1020_RESET_Pin, GPIO_PIN_RESET);
}

void myevk_radio_spi_set_cs(void)
{

    HAL_GPIO_WritePin(SR1020_CS_GPIO_Port,SR1020_CS_Pin, GPIO_PIN_SET);
}

void myevk_radio_spi_reset_cs(void)
{
    HAL_GPIO_WritePin(SR1020_CS_GPIO_Port,SR1020_CS_Pin, GPIO_PIN_RESET);
}

void myevk_radio_spi_transfer_full_duplex_blocking(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    HAL_SPI_TransmitReceive(&hspi2, tx_data,rx_data,size,10);
}

void myevk_radio_spi_transfer_full_duplex_non_blocking(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
#ifdef USE_ST_HAL_SPI_DRIVER
    myevk_radio_spi_reset_cs();
    HAL_SPI_TransmitReceive_DMA(&hspi2, tx_data,rx_data,size);
    __HAL_DMA_DISABLE_IT(&hradio_dma_spi_rx, DMA_IT_HT);

#else
    SCB_CleanDCache_by_Addr((uint32_t*)(((uint32_t)tx_data) & ~(uint32_t)0x1F),size + 32);

    hspi2.State = HAL_SPI_STATE_BUSY_TX_RX;
    myevk_radio_spi_reset_cs();
    // Enable SPI peripheral
    __HAL_SPI_DISABLE(&hspi2);
    DMA_HandleTypeDef *hdmarx = (&hspi2)->hdmarx;
    DMA_HandleTypeDef *hdmatx = (&hspi2)->hdmatx;

    // Reset the Tx/Rx DMA bits
    CLEAR_BIT((&hspi2)->Instance->CFG1, SPI_CFG1_TXDMAEN | SPI_CFG1_RXDMAEN);

    // Disable the peripheral
    __HAL_DMA_DISABLE(hdmarx);

    DMA_Base_Registers  *regs_dma  = (DMA_Base_Registers *)hdmarx->StreamBaseAddress;
    // Clear all interrupt flags at correct offset within the register
    regs_dma->IFCR = 0x3FUL << (hdmarx->StreamIndex & 0x1FU);

    // Clear DBM bit - No double buffering
    ((DMA_Stream_TypeDef *)hdmarx->Instance)->CR &= (uint32_t)(~DMA_SxCR_DBM);
    // Configure DMA Stream data length
    ((DMA_Stream_TypeDef *)hdmarx->Instance)->NDTR = size;
    // Configure DMA Stream source address
    ((DMA_Stream_TypeDef *)hdmarx->Instance)->PAR = (uint32_t)&(&hspi2)->Instance->RXDR;
        // Configure DMA Stream destination address
    ((DMA_Stream_TypeDef *)hdmarx->Instance)->M0AR = (uint32_t)rx_data;
    // Enable Common interrupts
    MODIFY_REG(((DMA_Stream_TypeDef *)hdmarx->Instance)->CR, (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_HT), (DMA_IT_TC | DMA_IT_TE | DMA_IT_DME));
    // Enable the Peripheral
    __HAL_DMA_ENABLE(hdmarx);

    // Enable Rx DMA Request
    SET_BIT((&hspi2)->Instance->CFG1, SPI_CFG1_RXDMAEN);

    __HAL_DMA_DISABLE(hdmatx);

    regs_dma  = (DMA_Base_Registers *)hdmatx->StreamBaseAddress;
    // Clear all interrupt flags at correct offset within the register
    regs_dma->IFCR = 0x3FUL << (hdmatx->StreamIndex & 0x1FU);

    // Configure DMA Stream data length
    ((DMA_Stream_TypeDef *)hdmatx->Instance)->NDTR = size;
    // Configure DMA Stream source address
    ((DMA_Stream_TypeDef *)hdmatx->Instance)->PAR = (uint32_t)&(&hspi2)->Instance->TXDR;

    // Configure DMA Stream destination address
    ((DMA_Stream_TypeDef *)hdmatx->Instance)->M0AR = (uint32_t)tx_data;
    // Enable the Peripheral
    __HAL_DMA_ENABLE(hdmatx);


    MODIFY_REG((&hspi2)->Instance->CR2, SPI_CR2_TSIZE, size);
    // Enable Tx DMA Request
    SET_BIT((&hspi2)->Instance->CFG1, SPI_CFG1_TXDMAEN);

    // Enable SPI peripheral
    __HAL_SPI_ENABLE(&hspi2);
    __HAL_SPI_ENABLE_IT(&hspi2,SPI_IT_EOT);

    // Master transfer start
    SET_BIT((&hspi2)->Instance->CR1, SPI_CR1_CSTART);
#endif
}

bool myevk_radio_is_spi_busy(void)
{
	return (&hspi2)->State == HAL_SPI_STATE_BUSY_TX_RX;
}

void myevk_radio_context_switch(void)
{
	EXTI->SWIER1 |= SR1020_INT_Pin; //Trigger radio interrupt by software
}

void myevk_radio_callback_context_switch(void)
{
	//Give semaphore to radio_context_switch
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(uwb_radio_callback_semHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


