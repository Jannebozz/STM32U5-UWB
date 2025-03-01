/** @file  circular_queue_critical_section.h
 *  @brief SPARK circular queue critical section macros definition.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef CIRCULAR_QUEUE_CRITICAL_SECTION_H_
#define CIRCULAR_QUEUE_CRITICAL_SECTION_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include "pulsar.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PRIVATE GLOBALS ************************************************************/
static uint32_t m_in_critical_region = 0;

/* MACROS *********************************************************************/
/** @brief Macro for entering a critical region.
 *
 *  @note This needs to handle nested interrupts.
 */
#define CRITICAL_SECTION_ENTER() { \
    __disable_irq(); \
    ++m_in_critical_region; \
}


/** @brief Macro for leaving a critical region.
 *
 *  @note This needs to handle nested interrupts.
 */
#define CRITICAL_SECTION_EXIT() { \
    if (--m_in_critical_region == 0) { \
        __enable_irq();\
    } \
}


#ifdef __cplusplus
}
#endif

#endif /* CIRCULAR_QUEUE_CRITICAL_SECTION_H_ */
