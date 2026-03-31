/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_host.h
  * @author  MCD Application Team
  * @brief   USBX Host applicative header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_USBX_HOST_H__
#define __APP_USBX_HOST_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "ux_api.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct
{
  ULONG otg_irq_count;
  ULONG otg_port_irq_count;
  ULONG hcd_connect_count;
  ULONG hcd_disconnect_count;
  ULONG usbx_change_count;
  ULONG usbx_error_count;
  ULONG last_usbx_event;
  ULONG last_gintsts;
  ULONG last_hprt0;
} APP_USBX_DIAG_SNAPSHOT;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
UINT MX_USBX_Host_Init(VOID *memory_ptr);

/* USER CODE BEGIN EFP */
VOID APP_USBX_Diag_RecordOtgIrq(ULONG gintsts, ULONG hprt0);
VOID APP_USBX_Diag_RecordHcdConnect(VOID);
VOID APP_USBX_Diag_RecordHcdDisconnect(VOID);
VOID APP_USBX_Diag_GetSnapshot(APP_USBX_DIAG_SNAPSHOT *snapshot);
VOID APP_USBX_Diag_PrintSnapshot(VOID);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOUSE_FLAG_CONNECTED        (1UL << 0)
#define MOUSE_FLAG_DISCONNECTED     (1UL << 1)
#define KEYBOARD_FLAG_CONNECTED     (1UL << 0)
#define KEYBOARD_FLAG_DISCONNECTED  (1UL << 1)
/* USER CODE END PD */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* __APP_USBX_HOST_H__ */
