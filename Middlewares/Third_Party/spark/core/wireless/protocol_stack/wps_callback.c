/** @file  wps_callback.c
 *  @brief The WPS callback module handle the callback queue of the wireless protocol stack.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "wps_callback.h"
#include "xlayer.h"

/* PUBLIC FUNCTIONS ***********************************************************/
void wps_callback_enqueue(circular_queue_t *queue, xlayer_t *xlayer)
{
    wps_callback_inst_t *callback;

    callback = circular_queue_get_free_slot(queue);
    if (callback != NULL) {
        callback->func = xlayer->config.callback;
        callback->parg = xlayer->config.parg_callback;
        circular_queue_enqueue(queue);
    }
}
