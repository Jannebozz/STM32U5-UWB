/** @file  myevk_timer.h
 *  @brief This module controls timer features of the SPARK EVK board.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef myevk_TIMER_H_
#define myevk_TIMER_H_

/* INCLUDES *******************************************************************/
#include "myevk_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Blocking delay with a 1ms resolution.
 */
void myevk_timer_delay_ms(uint32_t ms);

/** @brief Free running timer with a tick of 1 ms.
 *
 *  @return Tick count.
 */
uint64_t myevk_timer_get_free_running_tick_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* myevk_TIMER_H_ */

