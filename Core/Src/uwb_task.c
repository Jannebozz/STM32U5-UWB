
/** @file  hello_world_node.c
 *  @brief This is a basic example of how to use the SPARK Wireless Core.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* INCLUDES *******************************************************************/
#include "iface_wireless.h"
#include "swc_api.h"
#include "swc_cfg_node.h"
#include "swc_stats.h"
#include "stm32u5xx_hal.h"
#include "myevk_radio.h"
#include "uwb_task.h"

/* CONSTANTS *****************************************************
 **************/
#define SWC_MEM_POOL_SIZE     16000
typedef enum UXS_PacketType_t
{
	UXS_PKT_FB_TYPE = 0x54414c46,
} UXS_PacketType_t;

/* PRIVATE GLOBALS ************************************************************/
/* ** Wireless Core ** */
__attribute__((section (".d2"), aligned(4))) static uint8_t swc_memory_pool[SWC_MEM_POOL_SIZE];
static swc_hal_t hal;
static swc_node_t *node;
static swc_connection_t *rx_conn;
static swc_connection_t *tx_conn;

static uint32_t timeslot_us[] = SCHEDULE;
static uint32_t channel_sequence[] = CHANNEL_SEQUENCE;
static uint32_t channel_frequency[] = CHANNEL_FREQ;
static int32_t rx_timeslots[] = RX_TIMESLOTS;
static int32_t tx_timeslots[] = TX_TIMESLOTS;

/* ** Application Specific ** */
static uint32_t rx_count;

#define log_trace(MODULE,...)
/* PRIVATE FUNCTION PROTOTYPE *************************************************/
static void app_swc_core_init(swc_error_t *err);
static void conn_rx_success_callback(void *conn);
static void conn_tx_event_callback(void *conn);
static void conn_rx_event_callback(void *conn);
static void QueueEvent(UxsUwbEventItem_t* event);
bool UWB_Preprocesing(uint8_t * buf, size_t size);
UwbResult_t UXS_UwbDeinit();
UwbResult_t UXS_UwbInit();
static void UwbConnectionTask(void* argument);
static const char* get_event_name(UxsUwbEventId_t event);
/* Definitions for radio_callback_sem */
osSemaphoreId_t uwb_radio_callback_semHandle;
const osSemaphoreAttr_t uwb_radio_callback_sem_attributes = {
  .name = "uwb_radio_callback_sem"
};

/* Definitions for radioCallbackTask */
osThreadId_t uwb_radioCallbackTaskHandle;
const osThreadAttr_t uwb_radioCallbackTask_attributes = {
  .name = "uwb_radioCallbackTask",
  .stack_size = 4096 * 4,
  .priority = (osPriority_t) osPriorityRealtime1,
};

static QueueHandle_t EventQueue = NULL;
static volatile UxsUwbState_t state = UXS_UWB_DISCONNECTED;
SemaphoreHandle_t uwbSendThreadSem = NULL;
static TaskHandle_t uwbConnectionTaskHandle;

void UXS_Uwb_Init()
{
	EventQueue = xQueueCreate(1, sizeof(UxsUwbEventItem_t));
	vQueueAddToRegistry(EventQueue, "uxs_event_queue");
	uwbSendThreadSem = xSemaphoreCreateBinary();
	BaseType_t result = xTaskCreate(UwbConnectionTask,
			"UwbConnectionTask",
			512,
			NULL,
			osPriorityNormal,
			&uwbConnectionTaskHandle);
}

bool UXS_Uwb_Connect()
{
	log_trace(MODULE_UWB, "UXS_Uwb_Connect()");
	if (state == UXS_UWB_DISCONNECTED)
	{
		UxsUwbEventItem_t event = {.eventId = UXS_UWB_EVENT_CONNECT};
		QueueEvent(&event);
		return true;
	}
	return false;
}

bool UXS_Uwb_Disconnect()
{
	if (state == UXS_UWB_CONNECTED)
	{
		UxsUwbEventItem_t event = {.eventId = UXS_UWB_EVENT_DISCONNECT};
		QueueEvent(&event);
		return true;
	}
	return false;
}

static void QueueEvent(UxsUwbEventItem_t* event)
{
	if (xQueueOverwrite(EventQueue, event) != pdPASS )
	{
		assert(0);
	}
}

static void UwbConnectionTask(void* argument)
{
	UxsUwbEventItem_t event;
	while(1)
	{
		if (xQueueReceive(EventQueue, &event, portMAX_DELAY))
		{
			log_trace(MODULE_UWB, "UwbConnectionTask(). Received eventId: %s", get_event_name(event.eventId));
			switch(event.eventId)
			{
			case UXS_UWB_EVENT_CONNECT:
			{
				state = UXS_UWB_CONNECTING;
				if(UXS_UwbInit() == UXS_UWB_RES_OK)
				{
					state = UXS_UWB_CONNECTED;
				}
				else
				{
					state = UXS_UWB_DISCONNECTED;
				}
				break;
			}
			case UXS_UWB_EVENT_DISCONNECT:
			{
				state = UXS_UWB_DISCONNECTING;
				UXS_UwbDeinit();
				state = UXS_UWB_DISCONNECTED;
				break;
			}

			default:
				log_trace(MODULE_UWB, "UwbConnectionTask(). Unknown eventId: %d", event);
				break;
			}
			log_trace(MODULE_UWB, "UwbConnectionTask(). Event processed eventId: %s", get_event_name(event.eventId));
		}
	}
}

//Function that return the max link speed in kbps
uint32_t UXS_UWB_GetMaxLinkRate()
{
	//First, find out how much we can receive
	uint32_t schedule_time = 0;
	for(int i = 0; i < ARRAY_SIZE(timeslot_us); ++i)
	{
		schedule_time += timeslot_us[i];
	}
	uint32_t receiving = 0;
	for(int i = 0; i < ARRAY_SIZE(rx_timeslots); ++i)
	{
		receiving++;
	}
	float schedules_per_sec = (1000*1000)/(float)schedule_time;
	float receiving_per_sec = receiving*schedules_per_sec;
	float kpbs = 1E-3*8*MAX_PAYLOAD_SIZE_BYTE*receiving_per_sec;
	return (uint32_t)kpbs;

}

/* PUBLIC FUNCTIONS ***********************************************************/


bool UWB_Preprocesing(uint8_t * buf, size_t size)
{
	//We should always receive a abort in on packet.
	uint32_t abort_size = sizeof(UXS_PacketType_t);
	if(size != abort_size)
		return false;
	if(size < abort_size)
		return false;

	const int number = size - sizeof(UXS_PacketType_t);

	for(int i = 0; i <= number; ++i)
	{
		uint32_t *adress = (uint32_t*)buf;
		if(*adress == UXS_UWB_ABORT_CMD)
		{
			return true;
		}
		buf++;
	}
	return false;
}

void UWB_RadioCallbackTask(void *argument)
{
	for(;;)
	{
		xSemaphoreTake(uwb_radio_callback_semHandle, portMAX_DELAY);
		swc_connection_callbacks_processing_handler();
	}
}

void UWB_TX_LinkConnected()
{
	log_trace(MODULE_UWB, "UWB_TX_LinkConnected");
}

void UWB_TX_LinkDisconnected()
{
	log_trace(MODULE_UWB, "UWB_TX_LinkDisconnected");
}

void UWB_RX_LinkConnected()
{
	log_trace(MODULE_UWB, "UWB_RX_LinkConnected");
	//UxsEventItem_t event = {.eventId = UXS_EVENT_UWB_LINK_UP};
	//UXS_EventManager_HandleEvent(&event);
}

void UWB_RX_LinkDisconnected()
{
	log_trace(MODULE_UWB, "UWB_RX_LinkDisconnected");
	//UxsEventItem_t event = {.eventId = UXS_EVENT_UWB_LINK_DOWN};
	//UXS_EventManager_HandleEvent(&event);
}

UwbResult_t UXS_UwbInit()
{
	log_trace(MODULE_UWB, "UXS_UwbInit() started");
	uwb_radio_callback_semHandle = osSemaphoreNew(1, 0, &uwb_radio_callback_sem_attributes);
	uwb_radioCallbackTaskHandle = osThreadNew(UWB_RadioCallbackTask, NULL, &uwb_radioCallbackTask_attributes);

	//UXS_PH_SetPreprocessing(&UWB_Preprocesing);

	swc_error_t swc_err;
	log_trace(MODULE_UWB, "app_swc_core_init");
	app_swc_core_init(&swc_err);
	if (swc_err != SWC_ERR_NONE)
	{
		log_trace(MODULE_UWB, "Error init UWB hardware.");
		return UXS_UWB_HW_INIT_ERR;
	}
	log_trace(MODULE_UWB, "Connecting radio...");
	//set_tx_connection(tx_conn);
	swc_connect();
	//Always reset upon init of UWB.
	//UXS_UWB_ResetSendFrame();
	log_trace(MODULE_UWB, "UXS_UwbInit() done");
	xSemaphoreGive(uwbSendThreadSem);

	return UXS_UWB_RES_OK;
}

UwbResult_t UXS_UwbDeinit()
{
	//Wait for any ongoing sending to complete
	xSemaphoreTake(uwbSendThreadSem,portMAX_DELAY);
	log_trace(MODULE_UWB, "UXS_UwbDeinit() started");
	swc_disconnect();
	//set_tx_connection(tx_conn);
	//Server_UpdateConnectedStatus(false);
	if(uwb_radioCallbackTaskHandle)
	{
		vTaskDelete(uwb_radioCallbackTaskHandle);
		vSemaphoreDelete(uwb_radio_callback_semHandle);
		UXS_PH_SetPreprocessing(NULL);
	}
	xSemaphoreGive(uwbSendThreadSem);
	log_trace(MODULE_UWB, "UXS_UwbDeinit() done");

	return UXS_UWB_RES_OK;
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Initialize the Wireless Core.
 *
 *  @param[out] err  Wireless Core error code.
 */
static void app_swc_core_init(swc_error_t *err)
{
    iface_swc_hal_init(&hal);
    iface_swc_handlers_init();
#if (SWC_RADIO_COUNT == 2)
    iface_swc_dual_radio_timer_init();
#endif

    swc_cfg_t core_cfg = {
        .timeslot_sequence = timeslot_us,
        .timeslot_sequence_length = ARRAY_SIZE(timeslot_us),
        .channel_sequence = channel_sequence,
        .channel_sequence_length = ARRAY_SIZE(channel_sequence),
        .fast_sync_enabled = false,
        .random_channel_sequence_enabled = false,
        .memory_pool = swc_memory_pool,
        .memory_pool_size = SWC_MEM_POOL_SIZE
    };
    swc_init(core_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_node_cfg_t node_cfg = {
        .role = NETWORK_ROLE,
        .pan_id = PAN_ID,
        .coordinator_address = COORDINATOR_ADDRESS,
        .local_address = LOCAL_ADDRESS,
        .sleep_level = SWC_SLEEP_LEVEL
    };
    node = swc_node_init(node_cfg, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_radio_cfg_t radio_cfg = {
        .irq_polarity = IRQ_ACTIVE_HIGH,
        .std_spi = SPI_STANDARD
    };
    swc_node_add_radio(node, radio_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

#if (SWC_RADIO_COUNT == 2)
    swc_radio_cfg_t radio2_cfg = {
        .irq_polarity = IRQ_ACTIVE_HIGH,
        .std_spi = SPI_STANDARD
    };
    swc_node_add_radio(node, radio2_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }
#endif

    /* ** TX Connection ** */
    swc_connection_cfg_t tx_conn_cfg = {
        .name = "TX Connection",
        .source_address = LOCAL_ADDRESS,
        .destination_address = REMOTE_ADDRESS,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .queue_size = TX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = tx_timeslots,
        .timeslot_count = ARRAY_SIZE(tx_timeslots),
		.allocate_payload_memory = false,
        .ack_enabled = true,
		.arq_enabled = true,
        .arq_settings.try_count = 3,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = true,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    tx_conn = swc_connection_init(node, tx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t tx_channel_cfg = {
        .tx_pulse_count = TX_DATA_PULSE_COUNT,
        .tx_pulse_width = TX_DATA_PULSE_WIDTH,
        .tx_pulse_gain  = TX_DATA_PULSE_GAIN,
        .rx_pulse_count = RX_ACK_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        tx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(tx_conn, node, tx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    //swc_connection_set_tx_success_callback(tx_conn, send_tx_success_callback);
	//swc_connection_set_tx_dropped_callback(tx_conn, send_tx_drop_callback);
    swc_connection_set_event_callback(tx_conn, conn_tx_event_callback);

    /* ** RX Connection ** */
    swc_connection_cfg_t rx_conn_cfg = {
        .name = "RX Connection",
        .source_address = REMOTE_ADDRESS,
        .destination_address = LOCAL_ADDRESS,
        .max_payload_size = MAX_PAYLOAD_SIZE_BYTE,
        .queue_size = RX_DATA_QUEUE_SIZE,
        .modulation = SWC_MODULATION,
        .fec = SWC_FEC_LEVEL,
        .timeslot_id = rx_timeslots,
        .timeslot_count = ARRAY_SIZE(rx_timeslots),
        .allocate_payload_memory = true,
        .ack_enabled = true,
		.arq_enabled = true,
        .arq_settings.try_count = 3,
        .arq_settings.time_deadline = 0,
        .auto_sync_enabled = true,
        .cca_enabled = false,
        .throttling_enabled = false,
        .rdo_enabled = false,
        .fallback_enabled = false
    };
    rx_conn = swc_connection_init(node, rx_conn_cfg, &hal, err);
    if (*err != SWC_ERR_NONE) {
        return;
    }

    swc_channel_cfg_t rx_channel_cfg = {
        .tx_pulse_count = TX_ACK_PULSE_COUNT,
        .tx_pulse_width = TX_ACK_PULSE_WIDTH,
        .tx_pulse_gain  = TX_ACK_PULSE_GAIN,
        .rx_pulse_count = RX_DATA_PULSE_COUNT
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(channel_sequence); i++) {
        rx_channel_cfg.frequency = channel_frequency[i];
        swc_connection_add_channel(rx_conn, node, rx_channel_cfg, err);
        if (*err != SWC_ERR_NONE) {
            return;
        }
    }
    swc_connection_set_rx_success_callback(rx_conn, conn_rx_success_callback);
    swc_connection_set_event_callback(rx_conn, conn_rx_event_callback);

    swc_setup(node);
}

/** @brief Callback function when a frame has been successfully received.
 *
 *  @param[in] conn  Connection the callback function has been linked to.
 */
static void conn_rx_success_callback(void *conn)
{
	(void)conn;

	swc_error_t err;
	uint8_t *payload = NULL;
	/* Get new payload */
	uint8_t radio_data_frame_size = swc_connection_receive(rx_conn, &payload, &err);
	//UXS_PH_Receive(payload, radio_data_frame_size, &UXS_PH_ReceiveInfo);
	/* Free the payload memory */
	swc_connection_receive_complete(rx_conn, &err);
}
/** @brief Callback TX events.
 *
 *  @param[in] conn  Connection instance.
 */
static void conn_tx_event_callback(void *conn)
{
    switch (swc_get_event(conn)) {
        case SWC_EVENT_CONNECT:
            /* Handle connect event */
        	log_trace(MODULE_UWB, "Radio TX:\t%s","Slave TX Connected");
        	UWB_TX_LinkConnected();
            //iface_print_string();
            break;
        case SWC_EVENT_DISCONNECT:
            /* Handle disconnect event */
        	log_trace(MODULE_UWB, "Radio TX:\t%s", "Slave TX Disconnected");
        	UWB_TX_LinkDisconnected();
        	//iface_print_string("Slave TX Disonnected\n\r");
            break;
        case SWC_EVENT_ERROR:
            /* Handle error event */
        	log_trace(MODULE_UWB, "Radio TX:\t%s", "Slave TX Event error ");
        	//iface_print_string("Slave TX Event error \n\r");
            swc_get_event_error(conn);
            break;
        default:
            break;
    }
}

/** @brief Callback RX events.
 *
 *  @param[in] conn  Connection instance.
 */
static void conn_rx_event_callback(void *conn)
{
    switch (swc_get_event(conn)) {
        case SWC_EVENT_CONNECT:
            /* Handle connect event */
        	log_trace(MODULE_UWB, "Radio RX:\t%s","Slave RX Connected");
        	UWB_RX_LinkConnected();
            break;
        case SWC_EVENT_DISCONNECT:
            /* Handle disconnect event */
        	log_trace(MODULE_UWB, "Radio TX:\t%s", "Slave RX Disconnected");
        	UWB_RX_LinkDisconnected();
            break;
        case SWC_EVENT_ERROR:
            /* Handle error event */
        	log_trace(MODULE_UWB, "Radio TX:\t%s", "Slave RX Event error ");
        	//iface_print_string("Slave RX Event error \n\r");
        	swc_get_event_error(conn);
            break;
        default:
            break;
    }
}

#define CASE_RETURN_STR(enum_val)          case enum_val: return #enum_val;
static const char* get_event_name(UxsUwbEventId_t event)
{
    switch ((int)event)
    {
		CASE_RETURN_STR(UXS_UWB_EVENT_CONNECT)
		CASE_RETURN_STR(UXS_UWB_EVENT_DISCONNECT)
    }

    return "UNKNOWN_EVENT";
}
