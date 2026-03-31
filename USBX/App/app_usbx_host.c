/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_usbx_host.c
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
#include "ux_hcd_stm32.h"
#include "ux_host_class_storage.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"
#include "ux_host_class_hid_keyboard.h"
#include "ux_system.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Reduced to minimum for basic enumeration test */
#define UX_DEMO_MEMORY_SIZE     (64U * 1024U)
#define USBX_LOG_BUFFER_SIZE    256U
#define USBX_STRING_BUFFER_SIZE 64U
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static ULONG ux_demo_memory_buffer[UX_DEMO_MEMORY_SIZE / sizeof(ULONG)];
static ULONG error_counter;
static volatile APP_USBX_DIAG_SNAPSHOT usbx_diag_snapshot;

UX_HOST_CLASS_HID_MOUSE  *hid_mouse_instance = UX_NULL;
TX_EVENT_FLAGS_GROUP      mouse_event_flags;

static TX_THREAD          mouse_thread;
#define MOUSE_THREAD_STACK_SIZE  1024U
static UCHAR              mouse_thread_stack[MOUSE_THREAD_STACK_SIZE];

UX_HOST_CLASS_HID_KEYBOARD *hid_keyboard_instance = UX_NULL;
TX_EVENT_FLAGS_GROUP        keyboard_event_flags;

static TX_THREAD            keyboard_thread;
#define KEYBOARD_THREAD_STACK_SIZE  1024U
static UCHAR                keyboard_thread_stack[KEYBOARD_THREAD_STACK_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern HCD_HandleTypeDef hhcd_USB_OTG_FS;
extern UART_HandleTypeDef huart1;

static VOID error_handler(void);
static UINT usbx_host_change_callback(ULONG event, UX_HOST_CLASS *host_class, VOID *instance);
static VOID usbx_log_printf(const char *format, ...);
static VOID usbx_log_device_info(UX_DEVICE *device);
static const char *usbx_speed_to_string(ULONG speed);
static UINT usbx_get_device_string(UX_DEVICE *device, ULONG string_index, char *buffer, size_t buffer_size);
static VOID usbx_diag_record_change_event(ULONG event);
extern void hid_mouse_thread_entry(ULONG arg);
extern void hid_keyboard_thread_entry(ULONG arg);
/* USER CODE END PFP */
/**
  * @brief  Application USBX Host Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_USBX_Host_Init(VOID *memory_ptr)
{
  UINT ret = UX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_USBX_Host_MEM_POOL */
  (void)memory_ptr;
  UINT status = UX_SUCCESS;
  /* USER CODE END MX_USBX_Host_MEM_POOL */

  /* USER CODE BEGIN MX_USBX_Host_Init */
  usbx_log_printf("\r\n[USBX] Initializing...\r\n");

  status = ux_system_initialize((UCHAR *)ux_demo_memory_buffer, UX_DEMO_MEMORY_SIZE, UX_NULL, 0);
  if (status != UX_SUCCESS)
  {
    usbx_log_printf("[USBX] ux_system_initialize failed: 0x%02X\r\n", status);
    return status;
  }
  usbx_log_printf("[USBX] ux_system_initialize OK\r\n");

  status = ux_host_stack_initialize(usbx_host_change_callback);
  if (status != UX_SUCCESS)
  {
    usbx_log_printf("[USBX] ux_host_stack_initialize failed: 0x%02X\r\n", status);
    return status;
  }
  usbx_log_printf("[USBX] ux_host_stack_initialize OK\r\n");

  /* 注册 MSC 堆栈类 (U盘) */
  status = ux_host_stack_class_register(_ux_system_host_class_storage_name, ux_host_class_storage_entry);
  if (status != UX_SUCCESS)
  {
    usbx_log_printf("[USBX] MSC class register failed: 0x%02X\r\n", status);
    return status;
  }
  usbx_log_printf("[USBX] MSC class registered\r\n");

  /* 注册 HID 堆栈类 (鼠标/键盘) */
  status = ux_host_stack_class_register(_ux_system_host_class_hid_name, ux_host_class_hid_entry);
  if (status != UX_SUCCESS)
  {
    usbx_log_printf("[USBX] HID class register failed: 0x%02X\r\n", status);
    return status;
  }
  usbx_log_printf("[USBX] HID class registered\r\n");

  /* 注册 HID 客户端 (鼠标和键盘) */
  status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name,
                                              ux_host_class_hid_mouse_entry);
  if (status != UX_SUCCESS)
  {
    usbx_log_printf("[USBX] HID mouse client register failed: 0x%02X\r\n", status);
    return status;
  }
  usbx_log_printf("[USBX] HID mouse client registered\r\n");

  status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_keyboard_name,
                                              ux_host_class_hid_keyboard_entry);
  if (status != UX_SUCCESS)
  {
    usbx_log_printf("[USBX] HID keyboard client register failed: 0x%02X\r\n", status);
    return status;
  }
  usbx_log_printf("[USBX] HID keyboard client registered\r\n");

  /* 注册 HCD — 这一步会设置 hhcd->pData，让 HAL 回调能找到 USBX 数据结构 */
  status = ux_host_stack_hcd_register((UCHAR *)"stm32_otg_fs",
                                      ux_hcd_stm32_initialize,
                                      (ULONG)USB_OTG_FS_PERIPH_BASE,
                                      (ULONG)(ALIGN_TYPE)&hhcd_USB_OTG_FS);
  if (status != UX_SUCCESS)
  {
    usbx_log_printf("[USBX] ux_host_stack_hcd_register failed: 0x%02X\r\n", status);
    return status;
  }
  usbx_log_printf("[USBX] ux_host_stack_hcd_register OK (pData set)\r\n");

  /* Create mouse event flags and thread */
  status = tx_event_flags_create(&mouse_event_flags, "mouse_events");
  if (status != TX_SUCCESS)
  {
    usbx_log_printf("[USBX] mouse event flags create failed: 0x%02X\r\n", status);
    return UX_ERROR;
  }

  status = tx_thread_create(&mouse_thread, "mouse_thread",
                            hid_mouse_thread_entry, 0,
                            mouse_thread_stack, MOUSE_THREAD_STACK_SIZE,
                            20, 20,
                            TX_NO_TIME_SLICE, TX_AUTO_START);
  if (status != TX_SUCCESS)
  {
    usbx_log_printf("[USBX] mouse thread create failed: 0x%02X\r\n", status);
    return UX_ERROR;
  }
  usbx_log_printf("[USBX] Mouse thread created\r\n");

  /* Create keyboard event flags and thread */
  status = tx_event_flags_create(&keyboard_event_flags, "keyboard_events");
  if (status != TX_SUCCESS)
  {
    usbx_log_printf("[USBX] keyboard event flags create failed: 0x%02X\r\n", status);
    return UX_ERROR;
  }

  status = tx_thread_create(&keyboard_thread, "keyboard_thread",
                            hid_keyboard_thread_entry, 0,
                            keyboard_thread_stack, KEYBOARD_THREAD_STACK_SIZE,
                            20, 20,
                            TX_NO_TIME_SLICE, TX_AUTO_START);
  if (status != TX_SUCCESS)
  {
    usbx_log_printf("[USBX] keyboard thread create failed: 0x%02X\r\n", status);
    return UX_ERROR;
  }
  usbx_log_printf("[USBX] Keyboard thread created\r\n");

  /* ============================================================
   * 启动 USB Host 控制器 — 驱动 VBUS，激活连接检测。
   * 注意：必须在 HAL_NVIC_EnableIRQ 之前调用，这样当设备
   * 在上电前就已插好时，VBUS 供电可以先稳定，设备有充足的
   * 时间完成上电初始化（USB 规范要求至少 100ms）。
   * ============================================================ */
  if (HAL_HCD_Start(&hhcd_USB_OTG_FS) != HAL_OK)
  {
    usbx_log_printf("[USBX] HAL_HCD_Start failed\r\n");
    return UX_ERROR;
  }
  usbx_log_printf("[USBX] HAL_HCD_Start OK, VBUS driven\r\n");

  /* ============================================================
   * 关键：等待 200ms 让预插入的设备完成上电稳定。
   * 使用忙等循环，因为此时 ThreadX 调度器尚未启动，
   * 不能使用 tx_thread_sleep。
   * STM32F407 主频 168MHz，约 5 个周期/循环。
   * 200ms ≈ 168000000 * 0.2 / 5 ≈ 6720000 次。
   * ============================================================ */
  {
    volatile uint32_t wait;
    for (wait = 0; wait < 7000000UL; wait++)
    {
      __NOP();
    }
  }
  usbx_log_printf("[USBX] Attach debounce wait done\r\n");

  /* ============================================================
   * 现在才使能 OTG FS 中断！
   * hhcd->pData 已经被 ux_host_stack_hcd_register 设置好了，
   * VBUS 已稳定，预插入的设备已完成上电初始化，
   * 此时使能中断后的连接检测和枚举将正常进行。
   * ============================================================ */
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
  usbx_log_printf("[USBX] OTG_FS IRQ enabled\r\n");

  usbx_log_printf("\r\n[USBX] pool free: %lu / %lu bytes\r\n",
                  _ux_system->ux_system_regular_memory_pool_free,
                  _ux_system->ux_system_regular_memory_pool_size);
  usbx_log_printf("\r\nUSBX host started, waiting for USB device...\r\n");
  /* USER CODE END MX_USBX_Host_Init */

  return ret;
}

/* USER CODE BEGIN 1 */
VOID APP_USBX_Diag_RecordOtgIrq(ULONG gintsts, ULONG hprt0)
{
  usbx_diag_snapshot.otg_irq_count++;
  if ((gintsts & USB_OTG_GINTSTS_HPRTINT) != 0U)
  {
    usbx_diag_snapshot.otg_port_irq_count++;
  }
  usbx_diag_snapshot.last_gintsts = gintsts;
  usbx_diag_snapshot.last_hprt0 = hprt0;
}

VOID APP_USBX_Diag_RecordHcdConnect(VOID)
{
  usbx_diag_snapshot.hcd_connect_count++;
}

VOID APP_USBX_Diag_RecordHcdDisconnect(VOID)
{
  usbx_diag_snapshot.hcd_disconnect_count++;
}

VOID APP_USBX_Diag_GetSnapshot(APP_USBX_DIAG_SNAPSHOT *snapshot)
{
  if (snapshot == UX_NULL)
  {
    return;
  }

  *snapshot = usbx_diag_snapshot;
}

VOID APP_USBX_Diag_PrintSnapshot(VOID)
{
  APP_USBX_DIAG_SNAPSHOT snapshot;

  APP_USBX_Diag_GetSnapshot(&snapshot);
  usbx_log_printf("[USBX-DIAG] irq=%lu port_irq=%lu conn_cb=%lu disc_cb=%lu change=%lu err=%lu last_evt=0x%02lX GINTSTS=0x%08lX HPRT=0x%08lX\r\n",
                  snapshot.otg_irq_count,
                  snapshot.otg_port_irq_count,
                  snapshot.hcd_connect_count,
                  snapshot.hcd_disconnect_count,
                  snapshot.usbx_change_count,
                  snapshot.usbx_error_count,
                  snapshot.last_usbx_event,
                  snapshot.last_gintsts,
                  snapshot.last_hprt0);
}

static VOID usbx_diag_record_change_event(ULONG event)
{
  usbx_diag_snapshot.usbx_change_count++;
  usbx_diag_snapshot.last_usbx_event = event;
}

static VOID usbx_log_printf(const char *format, ...)
{
  va_list args;
  int length;
  char buffer[USBX_LOG_BUFFER_SIZE];

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length <= 0)
  {
    return;
  }

  if (length >= (int)sizeof(buffer))
  {
    length = (int)sizeof(buffer) - 1;
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, (uint16_t)length, HAL_MAX_DELAY);
}

static const char *usbx_speed_to_string(ULONG speed)
{
  switch (speed)
  {
    case UX_LOW_SPEED_DEVICE:
      return "low-speed";

    case UX_FULL_SPEED_DEVICE:
      return "full-speed";

    case UX_HIGH_SPEED_DEVICE:
      return "high-speed";

    default:
      return "unknown";
  }
}

static UINT usbx_get_device_string(UX_DEVICE *device, ULONG string_index, char *buffer, size_t buffer_size)
{
  UINT status;
  ULONG language_id;
  ULONG descriptor_length;
  ULONG raw_index;
  size_t text_index;
  UCHAR raw_descriptor[USBX_STRING_BUFFER_SIZE];

  if ((device == UX_NULL) || (buffer == NULL) || (buffer_size == 0U) || (string_index == 0U))
  {
    return UX_ERROR;
  }

  buffer[0] = '\0';

  status = ux_host_stack_device_string_get(device, raw_descriptor, sizeof(raw_descriptor), 0, 0);
  if ((status != UX_SUCCESS) || (raw_descriptor[0] < 4U))
  {
    return status;
  }

  language_id = (ULONG)raw_descriptor[2] | ((ULONG)raw_descriptor[3] << 8);

  status = ux_host_stack_device_string_get(device,
                                           raw_descriptor,
                                           sizeof(raw_descriptor),
                                           language_id,
                                           string_index);
  if ((status != UX_SUCCESS) || (raw_descriptor[0] < 2U))
  {
    return status;
  }

  descriptor_length = raw_descriptor[0];
  text_index = 0U;

  for (raw_index = 2U; (raw_index + 1U) < descriptor_length; raw_index += 2U)
  {
    unsigned int unicode_char = (unsigned int)raw_descriptor[raw_index] |
                                ((unsigned int)raw_descriptor[raw_index + 1U] << 8);

    if ((text_index + 1U) >= buffer_size)
    {
      break;
    }

    if ((unicode_char >= 32U) && (unicode_char <= 126U))
    {
      buffer[text_index++] = (char)unicode_char;
    }
    else if ((unicode_char == '\r') || (unicode_char == '\n') || (unicode_char == '\t'))
    {
      buffer[text_index++] = ' ';
    }
    else
    {
      buffer[text_index++] = '?';
    }
  }

  buffer[text_index] = '\0';
  return UX_SUCCESS;
}

static VOID usbx_log_device_info(UX_DEVICE *device)
{
  char manufacturer[USBX_STRING_BUFFER_SIZE];
  char product[USBX_STRING_BUFFER_SIZE];
  char serial[USBX_STRING_BUFFER_SIZE];
  UX_CONFIGURATION *configuration;
  UX_DEVICE_DESCRIPTOR *descriptor;

  if (device == UX_NULL)
  {
    usbx_log_printf("USB device info unavailable: device pointer is NULL\r\n");
    return;
  }

  descriptor = &device->ux_device_descriptor;

  usbx_log_printf("USB device connected\r\n");
  usbx_log_printf("  Address : %lu\r\n", device->ux_device_address);
  usbx_log_printf("  State   : %lu\r\n", device->ux_device_state);
  usbx_log_printf("  Speed   : %s\r\n", usbx_speed_to_string(device->ux_device_speed));
  usbx_log_printf("  VID:PID : %04lX:%04lX\r\n", descriptor->idVendor, descriptor->idProduct);
  usbx_log_printf("  USB Ver : %lx.%02lx\r\n",
                  ((descriptor->bcdUSB >> 8) & 0xFFUL),
                  ((((descriptor->bcdUSB >> 4) & 0x0FUL) * 10UL) | (descriptor->bcdUSB & 0x0FUL)));
  usbx_log_printf("  Dev Ver : %lx.%02lx\r\n",
                  ((descriptor->bcdDevice >> 8) & 0xFFUL),
                  ((((descriptor->bcdDevice >> 4) & 0x0FUL) * 10UL) | (descriptor->bcdDevice & 0x0FUL)));
  usbx_log_printf("  Class   : 0x%02lX / 0x%02lX / 0x%02lX\r\n",
                  descriptor->bDeviceClass,
                  descriptor->bDeviceSubClass,
                  descriptor->bDeviceProtocol);
  usbx_log_printf("  EP0 MPS : %lu\r\n", descriptor->bMaxPacketSize0);
  usbx_log_printf("  Configs : %lu\r\n", descriptor->bNumConfigurations);

  configuration = device->ux_device_current_configuration;
  if (configuration != UX_NULL)
  {
    UX_INTERFACE *iface;
    ULONG iface_idx = 0;

    usbx_log_printf("  Ifaces  : %lu\r\n", configuration->ux_configuration_descriptor.bNumInterfaces);
    usbx_log_printf("  Attr    : 0x%02lX\r\n", configuration->ux_configuration_descriptor.bmAttributes);
    usbx_log_printf("  MaxPwr  : %lu mA\r\n", configuration->ux_configuration_descriptor.MaxPower * 2UL);

    iface = configuration->ux_configuration_first_interface;
    while (iface != UX_NULL)
    {
      UX_ENDPOINT *ep;
      ULONG ep_idx = 0;

      usbx_log_printf("  IF[%lu]   : Class 0x%02lX / Sub 0x%02lX / Proto 0x%02lX  EPs:%lu\r\n",
                      iface_idx,
                      iface->ux_interface_descriptor.bInterfaceClass,
                      iface->ux_interface_descriptor.bInterfaceSubClass,
                      iface->ux_interface_descriptor.bInterfaceProtocol,
                      iface->ux_interface_descriptor.bNumEndpoints);

      ep = iface->ux_interface_first_endpoint;
      while (ep != UX_NULL)
      {
        ULONG addr = ep->ux_endpoint_descriptor.bEndpointAddress;
        ULONG attr = ep->ux_endpoint_descriptor.bmAttributes;
        const char *dir = (addr & 0x80U) ? "IN" : "OUT";
        const char *type;

        switch (attr & 0x03U)
        {
          case 0: type = "CTRL"; break;
          case 1: type = "ISOC"; break;
          case 2: type = "BULK"; break;
          case 3: type = "INT";  break;
          default: type = "?";   break;
        }

        usbx_log_printf("    EP[%lu] : 0x%02lX (%s %s)  MPS:%lu  Interval:%lu\r\n",
                        ep_idx, addr, dir, type,
                        ep->ux_endpoint_descriptor.wMaxPacketSize,
                        ep->ux_endpoint_descriptor.bInterval);
        ep = ep->ux_endpoint_next_endpoint;
        ep_idx++;
      }

      iface = iface->ux_interface_next_interface;
      iface_idx++;
    }
  }

  if (device->ux_device_state == UX_DEVICE_CONFIGURED)
  {
    if ((usbx_get_device_string(device, descriptor->iManufacturer, manufacturer, sizeof(manufacturer)) == UX_SUCCESS) &&
        (manufacturer[0] != '\0'))
    {
      usbx_log_printf("  Vendor  : %s\r\n", manufacturer);
    }

    if ((usbx_get_device_string(device, descriptor->iProduct, product, sizeof(product)) == UX_SUCCESS) &&
        (product[0] != '\0'))
    {
      usbx_log_printf("  Product : %s\r\n", product);
    }

    if ((usbx_get_device_string(device, descriptor->iSerialNumber, serial, sizeof(serial)) == UX_SUCCESS) &&
        (serial[0] != '\0'))
    {
      usbx_log_printf("  Serial  : %s\r\n", serial);
    }
  }
  else
  {
    usbx_log_printf("  Note    : device not fully configured, likely no matching USBX host class.\r\n");
  }
}

static UINT usbx_host_change_callback(ULONG event, UX_HOST_CLASS *host_class, VOID *instance)
{
  UX_DEVICE *device;

  (void)host_class;
  usbx_diag_record_change_event(event);

  switch (event)
  {
    case UX_DEVICE_CONNECTION:
      device = (UX_DEVICE *)instance;
      usbx_log_printf("\r\n[USBX] device connection event (state=%lu, pool_free=%lu)\r\n",
                      device != UX_NULL ? device->ux_device_state : 0UL,
                      _ux_system->ux_system_regular_memory_pool_free);
      usbx_log_device_info(device);
      break;

    case UX_DEVICE_DISCONNECTION:
      device = (UX_DEVICE *)instance;
      if (device != UX_NULL)
      {
        usbx_log_printf("\r\n[USBX] device disconnected, last VID:PID %04lX:%04lX\r\n",
                        device->ux_device_descriptor.idVendor,
                        device->ux_device_descriptor.idProduct);
      }
      else
      {
        usbx_log_printf("\r\n[USBX] device disconnected\r\n");
      }
      break;

    case UX_DEVICE_INSERTION:
      if (host_class != UX_NULL)
      {
        usbx_log_printf("[USBX] class activated: %s\r\n", host_class->ux_host_class_name);

        /* Check if this is a HID class with a mouse client */
        if (host_class->ux_host_class_entry_function == ux_host_class_hid_entry)
        {
          UX_HOST_CLASS_HID *hid = (UX_HOST_CLASS_HID *)instance;

          /* Print configured device info at INSERTION time (descriptor is valid now) */
          if (hid != UX_NULL && hid->ux_host_class_hid_interface != UX_NULL)
          {
            UX_DEVICE *dev = hid->ux_host_class_hid_interface
                                 ->ux_interface_configuration
                                 ->ux_configuration_device;
            usbx_log_device_info(dev);
          }

          if (hid != UX_NULL && hid->ux_host_class_hid_client != UX_NULL)
          {
            if (hid->ux_host_class_hid_client->ux_host_class_hid_client_handler == ux_host_class_hid_mouse_entry)
            {
              hid_mouse_instance = (UX_HOST_CLASS_HID_MOUSE *)hid->ux_host_class_hid_client->ux_host_class_hid_client_local_instance;
              usbx_log_printf("[USBX] HID Mouse instance captured\r\n");
              tx_event_flags_set(&mouse_event_flags, MOUSE_FLAG_CONNECTED, TX_OR);
            }
            else if (hid->ux_host_class_hid_client->ux_host_class_hid_client_handler == ux_host_class_hid_keyboard_entry)
            {
              hid_keyboard_instance = (UX_HOST_CLASS_HID_KEYBOARD *)hid->ux_host_class_hid_client->ux_host_class_hid_client_local_instance;
              usbx_log_printf("[USBX] HID Keyboard instance captured\r\n");
              tx_event_flags_set(&keyboard_event_flags, KEYBOARD_FLAG_CONNECTED, TX_OR);
            }
          }
        }
      }
      else
      {
        usbx_log_printf("[USBX] class activated (unknown)\r\n");
      }
      break;

    case UX_DEVICE_REMOVAL:
      if (host_class != UX_NULL)
      {
        usbx_log_printf("[USBX] class removed: %s\r\n", host_class->ux_host_class_name);

        /* Check if the removed class is a HID mouse */
        if (host_class->ux_host_class_entry_function == ux_host_class_hid_entry)
        {
          UX_HOST_CLASS_HID *hid = (UX_HOST_CLASS_HID *)instance;
          if (hid != UX_NULL && hid->ux_host_class_hid_client != UX_NULL)
          {
            if (hid->ux_host_class_hid_client->ux_host_class_hid_client_handler == ux_host_class_hid_mouse_entry)
            {
              hid_mouse_instance = UX_NULL;
              usbx_log_printf("[USBX] HID Mouse instance cleared\r\n");
              tx_event_flags_set(&mouse_event_flags, MOUSE_FLAG_DISCONNECTED, TX_OR);
            }
            else if (hid->ux_host_class_hid_client->ux_host_class_hid_client_handler == ux_host_class_hid_keyboard_entry)
            {
              hid_keyboard_instance = UX_NULL;
              usbx_log_printf("[USBX] HID Keyboard instance cleared\r\n");
              tx_event_flags_set(&keyboard_event_flags, KEYBOARD_FLAG_DISCONNECTED, TX_OR);
            }
          }
        }
      }
      else
      {
        usbx_log_printf("[USBX] class removed (unknown)\r\n");
      }
      break;

    default:
      break;
  }

  return UX_SUCCESS;
}

static VOID error_handler(void)
{
  error_counter++;
  usbx_diag_snapshot.usbx_error_count++;
  usbx_log_printf("[USBX] fatal error, counter=%lu\r\n", error_counter);

  while (1)
  {
  }
}
/* USER CODE END 1 */
