/** @file  swc_cfg_node.h
 *  @brief Application specific configuration constants for the SPARK Wireless Core.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SWC_CFG_NODE_H_
#define SWC_CFG_NODE_H_

/* CONSTANTS ******************************************************************/
#define MAX_PAYLOAD_SIZE_BYTE 125

#define NETWORK_ROLE        NETWORK_NODE
#define PAN_ID              0xBCD
#define COORDINATOR_ADDRESS 0x01
#define NODE_ADDRESS        0x02
#define LOCAL_ADDRESS       NODE_ADDRESS
#define REMOTE_ADDRESS      COORDINATOR_ADDRESS

/* Output power configuration */
#define TX_DATA_PULSE_COUNT 1
#define TX_DATA_PULSE_WIDTH 5
#define TX_DATA_PULSE_GAIN  0
#define TX_ACK_PULSE_COUNT  3
#define TX_ACK_PULSE_WIDTH  6
#define TX_ACK_PULSE_GAIN   0

/* Input power configuration */
#define RX_ACK_PULSE_COUNT  3 /* Pulses configuration of received ACK frames */
#define RX_DATA_PULSE_COUNT 1 /* Pulses configuration of received data frames */

/* SWC queue size */
#define TX_DATA_QUEUE_SIZE 10
#define RX_DATA_QUEUE_SIZE 10

/* Misc configurations */
#define SWC_MODULATION  MODULATION_IOOK
#define SWC_FEC_LEVEL   FEC_LVL_2
#define SWC_SLEEP_LEVEL SLEEP_IDLE

#ifdef DEBUG
#define TIMESLOT 500
#endif
#ifdef RELEASE
#define TIMESLOT 250
#endif

/* Schedule configuration */
#define SCHEDULE { \
    TIMESLOT, TIMESLOT, TIMESLOT, TIMESLOT, TIMESLOT, \
	TIMESLOT, TIMESLOT, TIMESLOT, TIMESLOT, TIMESLOT  \
}
#define TX_TIMESLOTS { \
   MAIN_TIMESLOT(9) \
}
#define RX_TIMESLOTS { \
    MAIN_TIMESLOT(0), MAIN_TIMESLOT(1), MAIN_TIMESLOT(2), MAIN_TIMESLOT(3),MAIN_TIMESLOT(4), \
    MAIN_TIMESLOT(5), MAIN_TIMESLOT(6), MAIN_TIMESLOT(7), MAIN_TIMESLOT(8) \
}


/* Channels */
#define CHANNEL_FREQ { \
    164, 171, 178, 185, 192 \
}
#define CHANNEL_SEQUENCE { \
    0, 1, 2, 3, 4 \
}

#endif /* SWC_CFG_NODE_H_ */
