/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_host_keyboard.c
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
#include "ux_host_class_hid_keyboard.h"
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
#define KB_LOG_BUFFER_SIZE  256U
#define KB_POLL_INTERVAL_MS 20U

/* Mask covering all individual modifier bits (left/right Ctrl, Shift, Alt, GUI) */
#define KB_MODIFIER_MASK  (UX_HID_KEYBOARD_STATE_LEFT_SHIFT  | UX_HID_KEYBOARD_STATE_RIGHT_SHIFT | \
                           UX_HID_KEYBOARD_STATE_LEFT_ALT    | UX_HID_KEYBOARD_STATE_RIGHT_ALT   | \
                           UX_HID_KEYBOARD_STATE_LEFT_CTRL   | UX_HID_KEYBOARD_STATE_RIGHT_CTRL  | \
                           UX_HID_KEYBOARD_STATE_LEFT_GUI    | UX_HID_KEYBOARD_STATE_RIGHT_GUI)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern UX_HOST_CLASS_HID_KEYBOARD *hid_keyboard_instance;
extern TX_EVENT_FLAGS_GROUP        keyboard_event_flags;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void  hid_keyboard_thread_entry(ULONG arg);
static void kb_log_printf(const char *format, ...);
static const char *kb_key_to_string(ULONG key, ULONG state);
static const char *kb_modifier_string(ULONG state, char *buf, size_t buf_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void kb_log_printf(const char *format, ...)
{
  extern UART_HandleTypeDef huart1;
  va_list args;
  int length;
  char buffer[KB_LOG_BUFFER_SIZE];

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length <= 0)
    return;

  if (length >= (int)sizeof(buffer))
    length = (int)sizeof(buffer) - 1;

  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
}

/* HID Usage ID to readable name (common keys) */
static const char *kb_key_to_string(ULONG key, ULONG state)
{
  /* If decoded key is printable ASCII (from USBX keyboard layout decode) */
  if ((key >= 0x20UL) && (key <= 0x7EUL))
  {
    return UX_NULL; /* Caller should print the char directly */
  }

  switch (key)
  {
    case 0x08: return "Backspace";
    case 0x09: return "Tab";
    case 0x0A: /* fall through */
    case 0x0D: return "Enter";
    case 0x1B: return "Escape";
    case 0x7F: return "Delete";
    default:   break;
  }

  /* Function key range: USBX decodes F1-F12 to 0x3A-0x45 usage IDs
     when keys_decode is disabled, or to function codes in state */
  if ((state & UX_HID_KEYBOARD_STATE_FUNCTION) != 0UL)
  {
    /* key is the raw HID usage */
    if (key >= 0x3AUL && key <= 0x45UL)
    {
      static char f_buf[4];
      f_buf[0] = 'F';
      if ((key - 0x3AUL + 1UL) >= 10UL)
      {
        f_buf[1] = '1';
        f_buf[2] = (char)('0' + (key - 0x3AUL + 1UL) - 10UL);
        f_buf[3] = '\0';
      }
      else
      {
        f_buf[1] = (char)('0' + (key - 0x3AUL + 1UL));
        f_buf[2] = '\0';
      }
      return f_buf;
    }
  }

  return "Special";
}

static const char *kb_modifier_string(ULONG state, char *buf, size_t buf_size)
{
  buf[0] = '\0';
  size_t pos = 0;

  if ((state & UX_HID_KEYBOARD_STATE_CTRL) != 0UL)
  {
    pos += (size_t)snprintf(buf + pos, buf_size - pos, "Ctrl+");
  }
  if ((state & UX_HID_KEYBOARD_STATE_ALT) != 0UL)
  {
    pos += (size_t)snprintf(buf + pos, buf_size - pos, "Alt+");
  }
  if ((state & UX_HID_KEYBOARD_STATE_SHIFT) != 0UL)
  {
    pos += (size_t)snprintf(buf + pos, buf_size - pos, "Shift+");
  }
  if ((state & UX_HID_KEYBOARD_STATE_GUI) != 0UL)
  {
    pos += (size_t)snprintf(buf + pos, buf_size - pos, "GUI+");
  }

  (void)pos;
  return buf;
}
/* Modifier bit -> readable name mapping (individual L/R keys) */
typedef struct {
  ULONG       bit;
  const char *name;
} KB_MOD_ENTRY;

static const KB_MOD_ENTRY kb_mod_table[] = {
  { UX_HID_KEYBOARD_STATE_LEFT_CTRL,   "L-Ctrl"  },
  { UX_HID_KEYBOARD_STATE_RIGHT_CTRL,  "R-Ctrl"  },
  { UX_HID_KEYBOARD_STATE_LEFT_SHIFT,  "L-Shift" },
  { UX_HID_KEYBOARD_STATE_RIGHT_SHIFT, "R-Shift" },
  { UX_HID_KEYBOARD_STATE_LEFT_ALT,    "L-Alt"   },
  { UX_HID_KEYBOARD_STATE_RIGHT_ALT,   "R-Alt"   },
  { UX_HID_KEYBOARD_STATE_LEFT_GUI,    "L-GUI"   },
  { UX_HID_KEYBOARD_STATE_RIGHT_GUI,   "R-GUI"   },
};
#define KB_MOD_TABLE_SIZE  (sizeof(kb_mod_table) / sizeof(kb_mod_table[0]))
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  hid_keyboard_thread_entry .
  * @param  ULONG arg
  * @retval Void
  */
void  hid_keyboard_thread_entry(ULONG arg)
{
  ULONG actual_flags;
  UINT  status;
  ULONG keyboard_key;
  ULONG keyboard_state;
  char  mod_buf[32];
  ULONG prev_mod_state;
  ULONG cur_mod_state;
  ULONG changed_bits;
  UINT  idx;

  (void)arg;

  kb_log_printf("\r\n[KB] Thread started, waiting for keyboard device...\r\n");

  while (1)
  {
    /* ---- Wait for keyboard connection ---- */
    status = tx_event_flags_get(&keyboard_event_flags,
                                KEYBOARD_FLAG_CONNECTED,
                                TX_OR_CLEAR,
                                &actual_flags,
                                TX_WAIT_FOREVER);
    if (status != TX_SUCCESS)
    {
      tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
      continue;
    }

    kb_log_printf("\r\n[KB] Keyboard connected, start reading keys...\r\n");

    /* Clear any stale DISCONNECTED flag left over from a previous unplug.
       Without this, a quick unplug-replug leaves bit 1 set, causing the
       inner loop to break out immediately on the very first poll cycle. */
    tx_event_flags_get(&keyboard_event_flags,
                       KEYBOARD_FLAG_DISCONNECTED,
                       TX_OR_CLEAR,
                       &actual_flags,
                       TX_NO_WAIT);

    prev_mod_state = 0;

    /* ---- Poll keyboard input until disconnected ---- */
    while (hid_keyboard_instance != UX_NULL)
    {
      /* Check for disconnect event (non-blocking) */
      status = tx_event_flags_get(&keyboard_event_flags,
                                  KEYBOARD_FLAG_DISCONNECTED,
                                  TX_OR_CLEAR,
                                  &actual_flags,
                                  TX_NO_WAIT);
      if (status == TX_SUCCESS)
      {
        kb_log_printf("[KB] Keyboard disconnected\r\n");
        break;
      }

      /* Try to get a key */
      keyboard_key = 0;
      keyboard_state = 0;
      status = ux_host_class_hid_keyboard_key_get(hid_keyboard_instance,
                                                   &keyboard_key,
                                                   &keyboard_state);
      if (status == UX_SUCCESS)
      {
        const char *is_up = (keyboard_state & UX_HID_KEYBOARD_STATE_KEY_UP) ? "UP" : "DOWN";
        const char *mod = kb_modifier_string(keyboard_state, mod_buf, sizeof(mod_buf));
        const char *name = kb_key_to_string(keyboard_key, keyboard_state);

        /* Lock key states */
        const char *caps = (keyboard_state & UX_HID_KEYBOARD_STATE_CAPS_LOCK) ? "CAPS " : "";
        const char *num  = (keyboard_state & UX_HID_KEYBOARD_STATE_NUM_LOCK)  ? "NUM "  : "";

        if (name == UX_NULL)
        {
          /* Printable ASCII character */
          kb_log_printf("[KB] %s %s'%c' (0x%02lX) [%s%s]\r\n",
                       is_up, mod, (char)keyboard_key, keyboard_key, caps, num);
        }
        else
        {
          /* Named special key */
          kb_log_printf("[KB] %s %s[%s] (0x%02lX) [%s%s]\r\n",
                       is_up, mod, name, keyboard_key, caps, num);
        }
      }
      else
      {
        /* No regular key — check if modifier state changed (standalone modifier press/release) */
        cur_mod_state = hid_keyboard_instance->ux_host_class_hid_keyboard_alternate_key_state & KB_MODIFIER_MASK;
        changed_bits  = cur_mod_state ^ prev_mod_state;
        if (changed_bits != 0UL)
        {
          for (idx = 0; idx < KB_MOD_TABLE_SIZE; idx++)
          {
            if ((changed_bits & kb_mod_table[idx].bit) != 0UL)
            {
              const char *action = (cur_mod_state & kb_mod_table[idx].bit) ? "DOWN" : "UP";
              kb_log_printf("[KB] %s [%s]\r\n", action, kb_mod_table[idx].name);
            }
          }
          prev_mod_state = cur_mod_state;
        }

        /* Sleep briefly */
        tx_thread_sleep(KB_POLL_INTERVAL_MS);
      }

      /* Keep modifier tracking in sync when a regular key is returned */
      if (status == UX_SUCCESS)
      {
        prev_mod_state = keyboard_state & KB_MODIFIER_MASK;
      }
    }

    kb_log_printf("[KB] Waiting for next keyboard device...\r\n");
  }
}

/* USER CODE END 1 */
