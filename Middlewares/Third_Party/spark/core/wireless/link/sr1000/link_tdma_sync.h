/** @file link_tdma_sync.h
 *  @brief TDMA sync module.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef LINK_TDMA_H_
#define LINK_TDMA_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "link_utils.h"
#include "link_cca.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CONSTANTS ******************************************************************/
#define UNSYNC_OFFSET_PLL_CYCLES 400

/* TYPES **********************************************************************/
#define STATE_SYNCING false
#define STATE_SYNCED  true

/** @brief Frame type enumeration.
 */
typedef enum frame_type {
    FRAME_RX, /**< Frame reception */
    FRAME_TX, /**< Frame transmission */
} frame_type_t;

typedef struct tdma_sync {
    sleep_lvl_t  sleep_mode;                       /**< Sleep mode */
    uint32_t     timeout_pll_cycles;               /**< timeout duration in PLL cycles */
    uint16_t     setup_time_pll_cycles;            /**< RX setup time in PLL cycles */
    uint16_t     base_target_rx_waited_pll_cycles; /**< Base target RX waited in PLL cycles */
    bool         slave_sync_state;                 /**< Synchronization state */
    uint16_t     frame_lost_max_count;             /**< Maximum consecutive lost frames before the sync is considered lost */
    uint32_t     sleep_offset_pll_cycles;          /**< Sleep time offset in PLL cycles */
    int32_t      sync_slave_offset;                /**< Slave sync off set in PLL cycles */
    frame_type_t previous_frame_type;              /**< Type of the previous frame */
    uint16_t     frame_lost_count;                 /**< Frame lost count */
    uint32_t     sleep_cycles_value;               /**< Sleep cycles value in PLL cycles */
    uint32_t     timeout_value;                    /**< Timeout value in PLL cycles */
    uint16_t     pwr_up_value;                     /**< Power up delay in PLL cycles */
    uint32_t     cca_unsync_watchdog_count;        /**< CCA unsync watch dog count */
    isi_mitig_t  isi_mitig;                        /**< ISI mitigation level (unused) */
} tdma_sync_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize TDMA sync object.
 *
 *                              |------preamble_size_bits------|-sync_word_size_bits-|
 *
 *  |---------------------------------------------timeout_pll_cycles---------------------------------------------|
 *
 *  |---setup_time_pll_cycles---|                                                    |---setup_time_pll_cycles---|
 *
 *
 *  The preamble and sync word are expected to arrive in the center of the RX timeout window.
 *
 *  In order for the frame to start within the RX timeout window, the setup time must be chosen to
 *  take into account the maximum possible drift between 2 receptions.
 *
 *
 *  @param[in] tdma_sync               TDMA sync object.
 *  @param[in] sleep_mode              Sleep mode.
 *  @param[in] setup_time_pll_cycles   RX setup time in PLL cycles.
 *  @param[in] frame_lost_max_count    Frame lost maximum count.
 *  @param[in] sync_word_size_bits     Sync word size in bits.
 *  @param[in] preamble_size_bits      Preamble size in bits.
 *  @param[in] pll_startup_xtal_cycles PLL startup time in XTAL cycles.
 *  @param[in] isi_mitig               ISI mitigation level. (unused)
 *  @param[in] isi_mitig_pauses        ISI mitigation level corresponding pauses.
 *  @param[in] fast_sync_enable        Fast sync enable flag.
 *  @return None.
 */
void link_tdma_sync_init(tdma_sync_t *tdma_sync,
                         sleep_lvl_t  sleep_mode,
                         uint16_t     setup_time_pll_cycles,
                         uint16_t     frame_lost_max_count,
                         uint8_t      sync_word_size_bits,
                         uint16_t     preamble_size_bits,
                         uint8_t      pll_startup_xtal_cycles,
                         isi_mitig_t  isi_mitig,
                         uint8_t      isi_mitig_pauses,
                         bool         fast_sync_enable);

/** @brief Update TDMA sync module for RX frame.
 *
 *  @param[in] tdma_sync            TDMA sync object.
 *  @param[in] duration_pll_cycles  Duration in PLL clock cycles.
 *  @param[in] cca                  CCA object.
 *  @return None.
 */
void link_tdma_sync_update_tx(tdma_sync_t *tdma_sync,
                              uint32_t     duration_pll_cycles,
                              link_cca_t  *cca);

/** @brief Update TDMA sync module for TX frame.
 *
 *  @param[in] tdma_sync            TDMA sync object.
 *  @param[in] duration_pll_cycles  Duration in PLL clock cycles.
 *  @param[in] cca                  CCA object.
 *  @return None.
 */
void link_tdma_sync_update_rx(tdma_sync_t *tdma_sync,
                              uint32_t     duration_pll_cycles,
                              link_cca_t  *cca);

/** @brief Update Adjust slave sync.
 *
 *  @param[in] tdma_sync             TDMA sync object.
 *  @param[in] frame_outcome         Frame outcome.
 *  @param[in] rx_waited_pll_cycles  RX waited value in PLL clock cycles.
 *  @param[in] cca                   CCA object.
 *  @param[in] rx_cca_retry_count    RX CCA retry count
 *  @return None.
 */
void link_tdma_sync_slave_adjust(tdma_sync_t    *tdma_sync,
                                 frame_outcome_t frame_outcome,
                                 uint16_t        rx_waited_pll_cycles,
                                 link_cca_t     *cca,
                                 uint8_t         rx_cca_retry_count);

/** @brief Try to get synced on the master
 *
 *  @param[in] tdma_sync             TDMA sync object.
 *  @param[in] frame_outcome         Frame outcome.
 *  @param[in] rx_waited_pll_cycles  RX waited value in PLL clock cycles.
 *  @param[in] cca                   CCA object.
 *  @param[in] rx_cca_retry_count    RX CCA retry count
 */
void link_tdma_sync_slave_find(tdma_sync_t    *tdma_sync,
                               frame_outcome_t frame_outcome,
                               uint16_t        rx_waited_pll_cycles,
                               link_cca_t     *cca,
                               uint8_t         rx_cca_retry_count);

/** @brief Get sleep cycles.
 *
 *  @param[in] tdma_sync  TDMA sync object.
 *  @return Sleep cycles.
 */
uint32_t link_tdma_sync_get_sleep_cycles(tdma_sync_t *tdma_sync);

/** @brief Get timeout.
 *
 *  @param[in] tdma_sync  TDMA sync object.
 *  @return Timeout.
 */
uint32_t link_tdma_sync_get_timeout(tdma_sync_t *tdma_sync);

/** @brief Get power up delay.
 *
 *  @param[in] tdma_sync  TDMA sync object.
 *  @return Power up delay.
 */
uint16_t link_tdma_sync_get_pwr_up(tdma_sync_t *tdma_sync);

/** @brief Get slave sync flag.
 *
 *  @param[in] tdma_sync  TDMA sync object.
 *  @return Slave sync state.
 */
bool link_tdma_sync_is_slave_synced(tdma_sync_t *tdma_sync);

/** @brief Get the ISI mitigation pauses.
 *
 *  @param[in] input_isi_mitig  Not implemented in SR10XX
 *  @return Number of ISI mitigation pauses
 */
uint8_t link_tdma_sync_get_isi_mitigation_pauses(isi_mitig_t isi_mitig_reg_val);

/** @brief Return the preamble length in bits for the MAC layer.
 *
 *  @note Feature not yet implemented in SR10XX.
 *
 *  @param[in] isi_mitig_pauses      Not implemented in SR10XX
 *  @param[in] preamble_len_reg_val  Preamble length register value.
 *  @return Preamble length in chips
 */
uint32_t link_tdma_get_preamble_length(uint8_t isi_mitig_pauses, uint32_t preamble_len_reg_val);

/** @brief Get syncword length.
 *
 *  @param[in] isi_mitig_pauses      Not implemented in SR10XX
 *  @param[in] syncword_len_reg_val  Syncword length register value.
 *  @return Syncword length in chips
 */
uint32_t link_tdma_get_syncword_length(uint8_t isi_mitig_pauses, syncword_length_t syncword_len_reg_val);


#ifdef __cplusplus
}
#endif
#endif /* LINK_MULTI_RADIO_H_ */
