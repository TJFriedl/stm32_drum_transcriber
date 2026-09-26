/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/p2p_server_app.c
  * @author  MCD Application Team
  * @brief   Peer to peer Server Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HELLO_MARKER  (0xA5)                                 /**< First byte of every "hello" packet */
#define HELLO_PERIOD  (1*1000*1000/CFG_TS_TICK_VAL)          /**< 1s between packets */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t HelloTimerId;
static uint8_t NotifyEnabled = 0;
static uint8_t HelloSeq = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void Hello_TimerCb(void);
static void Hello_Send(void);

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
      /* The client subscribed: start sending one packet per second */
      NotifyEnabled = 1;
      HelloSeq = 0;
      HW_TS_Start(HelloTimerId, HELLO_PERIOD);

/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
      NotifyEnabled = 0;
      HW_TS_Stop(HelloTimerId);

/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */

/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */

/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */

/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
    BSP_LED_On(LED_BLUE);

/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
    NotifyEnabled = 0;
    HW_TS_Stop(HelloTimerId);
    BSP_LED_Off(LED_BLUE);
    BSP_LED_Off(LED_GREEN);

/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */
  UTIL_SEQ_RegTask(1<<CFG_TASK_P2P_HELLO_ID, UTIL_SEQ_RFU, Hello_Send);
  HW_TS_Create(CFG_TIM_PROC_ID_ISR, &HelloTimerId, hw_ts_Repeated, Hello_TimerCb);

/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
/**
 * @brief  Timer callback (runs in interrupt context): just schedule the send task
 */
static void Hello_TimerCb(void)
{
  UTIL_SEQ_SetTask(1<<CFG_TASK_P2P_HELLO_ID, CFG_SCH_PRIO_0);
}

/**
 * @brief  Send one 2-byte "hello" notification: {HELLO_MARKER, sequence number}
 */
static void Hello_Send(void)
{
  uint8_t payload[2];

  if (NotifyEnabled == 0)
  {
    return;
  }

  payload[0] = HELLO_MARKER;
  payload[1] = HelloSeq;
  if (P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, payload) == BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : HELLO %02X %02X\n", payload[0], payload[1]);
    HelloSeq++;
    BSP_LED_Toggle(LED_GREEN);
  }
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/
