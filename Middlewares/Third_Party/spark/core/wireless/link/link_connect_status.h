/** @file link_connect_status.h
 *  @brief Link connection status module.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef LINK_CONNECT_STATUS_H_
#define LINK_CONNECT_STATUS_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "link_lqi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
typedef enum connect_status {
    CONNECT_STATUS_CONNECTED,   /**< Connected status */
    CONNECT_STATUS_DISCONNECTED /**< Disconnected status */
} connect_status_t;

typedef struct connect_status_cfg {
    uint8_t connect_count;    /**< Number of consecutive recieved frame before the status is changed to connected */
    uint8_t disconnect_count; /**< Number of consecutive lost frame before the status is changed to disconnected */
} connect_status_cfg_t;

typedef struct link_connect_status {
    uint8_t connect_count;    /**< Number of consecutive received frame before the status is changed to connected */
    uint8_t disconnect_count; /**< Number of consecutive lost frame before the status is changed to disconnected */
    uint8_t received_count;   /**< Current consecutive Received frame */
    uint8_t lost_count;       /**< Current consecutive lost frame */
    connect_status_t status;  /**< Current connection status */
} link_connect_status_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialise the link connection status module.
 *
 *  @param[in] cfg                  link connection status module configuration.
 *  @param[in] link_connect_status  link connection status module instance.
 */
void link_connect_status_init(connect_status_cfg_t *cfg, link_connect_status_t *link_connect_status);

/** @brief Initialise the link connection status module.
 *
 *  @param[in] link_connect_status link connection status module instance.
 *  @param[in] frame_outcome       Frame outcome.
 */
void link_update_connect_status(link_connect_status_t *link_connect_status, frame_outcome_t frame_outcome);

/** @brief Get the current connection status.
 *
 *  @param[in] link_connect_status Link connection status module instance.
 *  @param[out] current_status     Current connection status.
 *  @retval [true]  if the connection status has changed.
 *  @retval [false] if the connection status has nos changed.
 */
bool link_connect_get_status(link_connect_status_t *link_connect_status, connect_status_t *current_status);

#ifdef __cplusplus
}
#endif
#endif /* LINK_CONNECT_STATUS_H_ */
