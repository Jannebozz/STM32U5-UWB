/** @file  iface_swc_evk.c
 *  @brief This file contains the implementation of functions configuring the
 *         wireless core which calls the functions of the BSP of the EVK1.4.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_wireless.h"
#include "myevk_radio.h"
#include "myevk_it.h"
#include "myevk_timer.h"

/* PUBLIC FUNCTIONS ***********************************************************/
void iface_swc_hal_init(swc_hal_t *hal)
{
    hal->radio_hal[0].set_shutdown_pin   = myevk_radio_set_shutdown_pin;
    hal->radio_hal[0].reset_shutdown_pin = myevk_radio_reset_shutdown_pin;
    hal->radio_hal[0].set_reset_pin      = myevk_radio_set_reset_pin;
    hal->radio_hal[0].reset_reset_pin    = myevk_radio_reset_reset_pin;
    hal->radio_hal[0].read_irq_pin       = myevk_radio_read_irq_pin;
    hal->radio_hal[0].set_cs             = myevk_radio_spi_set_cs;
    hal->radio_hal[0].reset_cs           = myevk_radio_spi_reset_cs;
    hal->radio_hal[0].delay_ms           = myevk_timer_delay_ms;

    hal->radio_hal[0].transfer_full_duplex_blocking     = myevk_radio_spi_transfer_full_duplex_blocking;
    hal->radio_hal[0].transfer_full_duplex_non_blocking = myevk_radio_spi_transfer_full_duplex_non_blocking;
    hal->radio_hal[0].is_spi_busy                       = myevk_radio_is_spi_busy;
    hal->radio_hal[0].context_switch                    = myevk_radio_context_switch;
    hal->radio_hal[0].disable_radio_irq                 = myevk_radio_disable_irq_it;
    hal->radio_hal[0].enable_radio_irq                  = myevk_radio_enable_irq_it;
    hal->radio_hal[0].disable_radio_dma_irq             = myevk_radio_disable_dma_irq_it;
    hal->radio_hal[0].enable_radio_dma_irq              = myevk_radio_enable_dma_irq_it;

    hal->context_switch = myevk_radio_callback_context_switch;

    hal->get_tick_quarter_ms = myevk_timer_get_free_running_tick_ms;
}

void iface_swc_handlers_init(void)
{
    myevk_set_radio_irq_callback(swc_radio_irq_handler);
    myevk_set_radio_dma_rx_callback(swc_radio_spi_receive_complete_handler);
}
