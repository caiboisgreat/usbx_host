/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_usbx_host.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define TX_THREAD_STACK_SIZE      2048U
TX_THREAD tx_testthread;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void tx_testthread_entry(ULONG args);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
  CHAR *pointer = NULL;
  ret = tx_byte_allocate(byte_pool, (VOID **) &pointer, TX_THREAD_STACK_SIZE, TX_NO_WAIT);
  if(ret != TX_SUCCESS){
	  return ret;
  }

  ret = (UINT)tx_thread_create(&tx_testthread,
                               "ThreadX Test",
                               tx_testthread_entry,
                               0,
                               pointer,
                               TX_THREAD_STACK_SIZE,
                               24, 24,
                               TX_NO_TIME_SLICE,
                               TX_AUTO_START);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */
void tx_testthread_entry(ULONG args) {
  extern UART_HandleTypeDef huart1;
  (void)args;

  /* Confirm the ThreadX scheduler started and this thread is running */
  {
    const char msg[] = "\r\n[TX] Test thread started OK\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
  }

  /* One-time USB register dump to verify hardware state */
  {
    char buf[256];
    uint32_t usb_base = (uint32_t)USB_OTG_FS;
    volatile uint32_t *hprt = (volatile uint32_t *)(usb_base + 0x440U);
    int len = snprintf(buf, sizeof(buf),
                       "[TX] USB regs: GCCFG=0x%08lX GINTSTS=0x%08lX GINTMSK=0x%08lX HPRT=0x%08lX GAHBCFG=0x%08lX GUSBCFG=0x%08lX\r\n",
                       (unsigned long)USB_OTG_FS->GCCFG,
                       (unsigned long)USB_OTG_FS->GINTSTS,
                       (unsigned long)USB_OTG_FS->GINTMSK,
                       (unsigned long)*hprt,
                       (unsigned long)USB_OTG_FS->GAHBCFG,
                       (unsigned long)USB_OTG_FS->GUSBCFG);
    if (len > 0 && len < (int)sizeof(buf))
      HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
  }

  while (1) {
    HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    HAL_Delay(100U);
  }
}
/* USER CODE END 1 */
