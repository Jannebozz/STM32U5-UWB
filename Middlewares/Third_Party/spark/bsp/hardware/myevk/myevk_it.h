/** @file  myevk_it.h
 *  @brief This module controls interrupt related features.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef myevk_IT_H_
#define myevk_IT_H_

/* INCLUDES *******************************************************************/
#include "myevk_def.h"

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t radio_interrupt_enabled;
/* TYPES **********************************************************************/
/** @brief Interrupt's module function callback type.
 */
typedef void (*irq_callback)(void);

void DMA_SPI_Rx_Interrupt(void);
void DMA_SPI_Tx_Interrupt(void);
void Radio_IRQ_Handler(void);
/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief This function set the function callback for the radio pin interrupt.
 *
 *  @param[in] callback  External interrupt callback function pointer.
 */
void myevk_set_radio_irq_callback(irq_callback callback);

/** @brief This function sets the function callback for the DMA_RX ISR.
 *
 *  @param[in] callback  External interrupt callback function pointer.
 */
void myevk_set_radio_dma_rx_callback(irq_callback callback);

#ifdef __cplusplus
}
#endif

#endif /* myevk_IT_H_ */

