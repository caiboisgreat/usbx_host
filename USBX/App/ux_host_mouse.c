/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_host_mouse.c
  * @author  MCD Application Team
  * @brief   USBX host applicative file
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

/* Includes ------------------------------------------------------------------*/
#include "app_usbx_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"
#include "tx_api.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOUSE_LOG_BUFFER_SIZE  256U
#define MOUSE_POLL_INTERVAL_MS 50U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern UX_HOST_CLASS_HID_MOUSE  *hid_mouse_instance;
extern TX_EVENT_FLAGS_GROUP      mouse_event_flags;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void  hid_mouse_thread_entry(ULONG arg);
static void mouse_log_printf(const char *format, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void mouse_log_printf(const char *format, ...)
{
  extern UART_HandleTypeDef huart1;
  va_list args;
  int length;
  char buffer[MOUSE_LOG_BUFFER_SIZE];

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length <= 0)
    return;

  if (length >= (int)sizeof(buffer))
    length = (int)sizeof(buffer) - 1;

  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
}
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  hid_mouse_thread_entry .
  * @param  ULONG arg
  * @retval Void
  */
void  hid_mouse_thread_entry(ULONG arg)
{
  ULONG actual_flags;
  UINT  status;
  ULONG buttons;
  ULONG prev_buttons = 0;
  SLONG mouse_x, mouse_y;
  SLONG mouse_wheel;
  SLONG accumulated_x = 0, accumulated_y = 0;

  (void)arg;

  mouse_log_printf("\r\n[MOUSE] Thread started, waiting for mouse device...\r\n");

  while (1)
  {
    /* ---- Wait for mouse connection ---- */
    status = tx_event_flags_get(&mouse_event_flags,
                                MOUSE_FLAG_CONNECTED,
                                TX_OR_CLEAR,
                                &actual_flags,
                                TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
      continue;
    }

    mouse_log_printf("\r\n[MOUSE] Mouse connected, start reading input...\r\n");
    prev_buttons = 0;
    accumulated_x = 0;
    accumulated_y = 0;

    /* ---- Poll mouse input until disconnected ---- */
    while (hid_mouse_instance != UX_NULL)
    {
      /* Check for disconnect event (non-blocking) */
      status = tx_event_flags_get(&mouse_event_flags,
                                  MOUSE_FLAG_DISCONNECTED,
                                  TX_OR_CLEAR,
                                  &actual_flags,
                                  TX_NO_WAIT);
      if (status == TX_SUCCESS)
      {
        mouse_log_printf("[MOUSE] Mouse disconnected\r\n");
        break;
      }

      /* Read button state */
      buttons = 0;
      status = ux_host_class_hid_mouse_buttons_get(hid_mouse_instance, &buttons);
      if (status != UX_SUCCESS)
      {
        tx_thread_sleep(MOUSE_POLL_INTERVAL_MS);
        continue;
      }

      /* Read position delta */
      mouse_x = 0;
      mouse_y = 0;
      ux_host_class_hid_mouse_position_get(hid_mouse_instance, &mouse_x, &mouse_y);

      /* Read wheel delta */
      mouse_wheel = 0;
      ux_host_class_hid_mouse_wheel_get(hid_mouse_instance, &mouse_wheel);

      /* Accumulate absolute position for tracking */
      accumulated_x += mouse_x;
      accumulated_y += mouse_y;

      /* Report button changes */
      if (buttons != prev_buttons)
      {
        ULONG changed = buttons ^ prev_buttons;

        if (changed & 0x01UL)
        {
          mouse_log_printf("[MOUSE] Left button %s\r\n",
                          (buttons & 0x01UL) ? "PRESSED" : "RELEASED");
        }
        if (changed & 0x02UL)
        {
          mouse_log_printf("[MOUSE] Right button %s\r\n",
                          (buttons & 0x02UL) ? "PRESSED" : "RELEASED");
        }
        if (changed & 0x04UL)
        {
          mouse_log_printf("[MOUSE] Middle button %s\r\n",
                          (buttons & 0x04UL) ? "PRESSED" : "RELEASED");
        }

        mouse_log_printf("[MOUSE] Buttons: L=%lu R=%lu M=%lu\r\n",
                        (buttons & 0x01UL), ((buttons >> 1) & 0x01UL), ((buttons >> 2) & 0x01UL));
        prev_buttons = buttons;
      }

      /* Report movement (only if there is actual movement) */
      if ((mouse_x != 0) || (mouse_y != 0))
      {
        mouse_log_printf("[MOUSE] Move: dx=%ld dy=%ld  Pos: X=%ld Y=%ld\r\n",
                        (long)mouse_x, (long)mouse_y,
                        (long)accumulated_x, (long)accumulated_y);
      }

      /* Report wheel (only if there is actual movement) */
      if (mouse_wheel != 0)
      {
        mouse_log_printf("[MOUSE] Wheel: %ld\r\n", (long)mouse_wheel);
      }

      tx_thread_sleep(MOUSE_POLL_INTERVAL_MS);
    }

    mouse_log_printf("[MOUSE] Waiting for next mouse device...\r\n");
  }
}

/* USER CODE END 1 */
