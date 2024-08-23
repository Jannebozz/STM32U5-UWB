/** @file  myevk_def.h
 *  @brief This defines MCU pinout and other control definitions.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef myevk_DEF_H_
#define myevk_DEF_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "main.h" 				//To get access to pin definitions
#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MACROS *********************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*a))
//Comment this to use optimized SPI Driver
#define USE_ST_HAL_SPI_DRIVER

#ifdef __cplusplus
}
#endif

#endif /* myevk_DEF_H_ */

