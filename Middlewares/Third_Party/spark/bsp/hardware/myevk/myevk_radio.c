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
#include "../../../Middlewares/Third_Party/FreeRTOS/Source/include/task.h"
/* PRIVATE GLOBALS ************************************************************/

extern DMA_HandleTypeDef handle_GPDMA1_Channel3;

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
	//HAL_NVIC_EnableIRQ(EXTI6_IRQn);
	SET_BIT(EXTI->IMR1, SR1020_INT_Pin);
}

void myevk_radio_disable_irq_it(void)
{
	//Mask interrupt
	CLEAR_BIT(EXTI->IMR1, SR1020_INT_Pin);
	//Clear any pending
	SET_BIT(EXTI->RPR1, SR1020_INT_Pin);
}
void myevk_radio_enable_dma_irq_it(void)
{
	//This should be DMA RX channel
    NVIC_EnableIRQ(GPDMA1_Channel3_IRQn);
}

void myevk_radio_disable_dma_irq_it(void)
{
	//This should be DMA RX channel
    NVIC_DisableIRQ(GPDMA1_Channel3_IRQn);
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
    HAL_SPI_TransmitReceive(&hspi1, tx_data,rx_data,size,10);
}

void myevk_radio_spi_transfer_full_duplex_non_blocking(uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
#ifdef USE_ST_HAL_SPI_DRIVER
    HAL_SPI_TransmitReceive_DMA(&hspi1, tx_data,rx_data,size);
    __HAL_DMA_DISABLE_IT(&handle_GPDMA1_Channel3, DMA_IT_HT); //Disable HalfTransfer interrupt
#else
	#error "This is not implemented for STM32U5 yet"
#endif
}

bool myevk_radio_is_spi_busy(void)
{
	return (&hspi1)->State == HAL_SPI_STATE_BUSY_TX_RX;
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


