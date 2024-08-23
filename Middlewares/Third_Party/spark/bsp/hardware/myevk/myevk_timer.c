/** @file  myevk_timer.c
 *  @brief This module controls timer features of the SPARK EVK board.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "myevk_timer.h"
#include "cmsis_os2.h"

/* CONSTANT *******************************************************************/
#define MAX_VALUE_UINT16_T  0xffff
#define APP_TIMER_PRESCALER 3
#define SYSTICK_TIMER_PRESCALER 3
#define SYSTICK_TIMER_FREQ 1000

/* PRIVATE GLOBALS ************************************************************/

/* PUBLIC FUNCTIONS ***********************************************************/


void myevk_timer_delay_ms(uint32_t ms)
{
	osDelay(ms);
}

uint64_t myevk_timer_get_free_running_tick_ms(void)
{
	return 0;
}
