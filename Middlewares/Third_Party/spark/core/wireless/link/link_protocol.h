/** @file link_protocol.h
 *  @brief WPS layer 2 internal connection protocol.
 *
 *  This file is a wrapper use to send/received payload
 *  through the WPS L2 internal connection. Its used to
 *  properly generate a complete packet regrouping one
 *  or multiple information.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef LINK_PROTOCOL_H_
#define LINK_PROTOCOL_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "link_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define MAX_NUMBER_OF_PROTOCOL 10

/* TYPES **********************************************************************/
/** @brief Link protocol instance configuration structure.
 */
typedef struct link_protocol_init_cfg {
    uint16_t buffer_size;  /**< Transmission buffer size, this should contains every protocol memory */
} link_protocol_init_cfg_t;

/** @brief Protocol configuration structure.
 *
 *  @note This should be provided when adding a protocol
 *        to the link_protocol instance.
 */
typedef struct link_protocol_cfg {
    void *instance;                                  /**< Protocol object. */
    uint8_t size;                                    /**< Send/Receive size use by the protocol. Size should
                                                       *  be the same for RX/TX.
                                                       */
    void (*send)(void *self, uint8_t *tx_buffer);    /**< Protocol transmit function, this function should
                                                       *  populate the tx_buffer provided based on the init
                                                       *  size.
                                                       */
    void (*receive)(void *self, uint8_t *rx_buffer); /**< Protocol receive function, this function should
                                                       *  extract the data from the received payload.
                                                       */
} link_protocol_cfg_t;

/** @brief Protocol internal info for link_protocol module.
 */
typedef struct link_protocol_internal_info {
    void *instance;                                  /**< Protocol object. */
    uint8_t index;                                   /**< Protocol buffer offset, given by the link_protocol for each protocol.
                                                       *  to write/read their buffer.
                                                       */
    uint32_t size;                                   /**< Protocol RX/TX size. */
    void (*send)(void *self, uint8_t *tx_buffer);    /**< Protocol transmit function, this function should
                                                       *  populate the tx_buffer provided based on the init
                                                       *  size.
                                                       */
    void (*receive)(void *self, uint8_t *rx_buffer); /**< Protocol receive function, this function should
                                                       *  extract the data from the received payload.
                                                       */
} link_protocol_internal_info_t;

/** @brief Link protocol instance.
 */
typedef struct link_protocol {
    uint8_t   index;                                                     /**< Transmission buffer that encapsulate every protocol. */
    uint8_t   current_buffer_offset;                                     /**< Buffer offset use to know where every protocol put their data. */
    uint8_t   current_number_of_protocol;                                /**< Total number of protocol. */
    uint16_t  max_buffer_size;                                           /**< Total protocol buffer size. */
    link_protocol_internal_info_t protocol_info[MAX_NUMBER_OF_PROTOCOL]; /**< Internal protocol info array. */
} link_protocol_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the link protocol instance.
 *
 *  @note Only the protocol buffer is needed here.
 *        Every protocol should be added using the
 *        link_protocol_add function.
 *
 *  @param[in]  link_protocol      Link protocol instance.
 *  @param[in]  link_protocol_cfg  Link protocol init configuration instance.
 *  @param[out] err                Link error instance.
 */
void link_protocol_init(link_protocol_t *link_protocol, link_protocol_init_cfg_t *link_protocol_cfg, link_error_t *err);

/** @brief Add a protocol to the link protocol.
 *
 *  @note  RX/TX size of the given protocol should be the same.
 *
 *  @param[in]  link_protocol  Link protocol instance.
 *  @param[in]  protocol_cfg   Protocol configuration instance.
 *  @param[out] err            Link error instance.
 */
void link_protocol_add(link_protocol_t *link_protocol, link_protocol_cfg_t *protocol_cfg, link_error_t *err);

/** @brief Populate the given TX buffer with all the protocol.
 *
 *  *  @note This will call every protocols send function with
 *           their respective buffer in order for them to properly
 *           populate their data.
 *
 *  @param[in]  link_protocol   Link protocol instance.
 *  @param[out] buffer_to_send  Pointer to the buffer that will be sent.
 *  @param[out] size            Size of the buffer to send.
 */
void link_protocol_send_buffer(void *link_protocol, uint8_t *buffer_to_send, uint32_t *size);

/** @brief Receive a giving buffer with all the protocol.
 *
 *  @note This will call every protocols receive function with
 *        their respective buffer in order for them to properly
 *        extract their data.
 *
 *  @param[in] link_protocol    Link protocol instance.
 *  @param[in] received_buffer  Buffer to received.
 *  @param[in] size             Size of the received buffer.
 */
void link_protocol_receive_buffer(void *link_protocol, uint8_t *received_buffer, uint32_t size);

#ifdef __cplusplus
}
#endif
#endif /* LINK_PROTOCOL_H_ */
