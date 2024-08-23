
#ifndef CLIENT_WATCH_CM7_APP_INC_UWB_TASK_H_
#define CLIENT_WATCH_CM7_APP_INC_UWB_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
#include "stdbool.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"
/* USER CODE END Includes */

typedef enum UwbResult_t
{
	UXS_UWB_RES_OK = 1,
	UXS_UWB_HW_INIT_ERR = -1,
	UXS_UWB_STACK_DEINIT_ERROR = -2,
	UXS_UWB_RES_SEND_ERROR = -3,
	UXS_UWB_RES_NOT_CONNECTED_ERROR = -4,
} UwbResult_t;

typedef enum {
    UXS_UWB_EVENT_CONNECT = 0,
	UXS_UWB_EVENT_DISCONNECT,
	UXS_UWB_EVENT_SCAN,
} UxsUwbEventId_t;

typedef struct
{
	UxsUwbEventId_t eventId;
	uint32_t data;
} UxsUwbEventItem_t;

typedef enum {
	UXS_UWB_DISCONNECTED = 0,
	UXS_UWB_CONNECTING,
	UXS_UWB_CONNECTED,
	UXS_UWB_DISCONNECTING,
} UxsUwbState_t;

#define UXS_UWB_ABORT_CMD 0x54524241

void UXS_Uwb_Init();
bool UXS_Uwb_Connect();
bool UXS_Uwb_Disconnect();

uint32_t UXS_UWB_GetMaxLinkRate();
extern osSemaphoreId_t uwb_radio_callback_semHandle;
extern SemaphoreHandle_t uwbSendThreadSem;
#endif // CLIENT_WATCH_CM7_APP_INC_UWB_TASK_H_
