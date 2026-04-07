/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_host_msc.c
  * @author  MCD Application Team
  * @brief   USBX host applicative file — MSC (Mass Storage Class) 处理
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
#include "ux_api.h"
#include "ux_host_class_storage.h"
#include "fx_api.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern UINT _fx_partition_offset_calculate(void *partition_sector, UINT partition,
                                           ULONG *partition_start, ULONG *partition_size);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSC_LOG_BUF_SIZE       256
#define MSC_MEDIA_BUF_SIZE     2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern TX_EVENT_FLAGS_GROUP   msc_event_flags;
extern UX_HOST_CLASS_STORAGE  *msc_storage_instance;
extern UART_HandleTypeDef     huart1;

static FX_MEDIA  msc_media;
static UCHAR     msc_media_buffer[MSC_MEDIA_BUF_SIZE] __attribute__((aligned(4)));

static ULONG msc_partition_start;   /* 分区起始扇区号 (MBR 偏移) */
static ULONG msc_partition_size;    /* 分区扇区数 */
static UINT  msc_active_lun;        /* 当前活动 LUN */
static ULONG msc_sector_size;       /* 当前扇区大小 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void  msc_process_thread_entry(ULONG arg);
static void  msc_log(const char *fmt, ...);
static UINT  msc_scsi_read_capacity(UX_HOST_CLASS_STORAGE *storage,
                                     ULONG *p_last_sector, ULONG *p_sector_size);
static void  msc_fx_driver(FX_MEDIA *media_ptr);
static void  msc_print_media_info(FX_MEDIA *media);
static UINT  msc_select_cluster_size(ULONG total_sectors, ULONG sector_size);
static UINT  msc_detect_partition(UX_HOST_CLASS_STORAGE *storage,
                                   ULONG total_sectors, ULONG sector_size);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  通过 UART 打印日志
  */
static void msc_log(const char *fmt, ...)
{
  char buf[MSC_LOG_BUF_SIZE];
  va_list args;
  int len;

  va_start(args, fmt);
  len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (len > 0)
  {
    if (len >= (int)sizeof(buf))
      len = (int)sizeof(buf) - 1;
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, HAL_MAX_DELAY);
  }
}

/**
  * @brief  通过 SCSI READ CAPACITY 命令获取磁盘容量
  * @note   调用前需持有 storage semaphore
  * @retval UX_SUCCESS 成功; 其他值表示失败
  */
static UINT msc_scsi_read_capacity(UX_HOST_CLASS_STORAGE *storage,
                                    ULONG *p_last_sector, ULONG *p_sector_size)
{
  UCHAR *cbw;
  UCHAR *response;
  UINT   status;
  ULONG  sense;

  response = _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY,
                                          UX_HOST_CLASS_STORAGE_READ_CAPACITY_RESPONSE_LENGTH);
  if (response == UX_NULL)
    return UX_MEMORY_INSUFFICIENT;

  cbw = (UCHAR *)storage->ux_host_class_storage_cbw;

  _ux_host_class_storage_cbw_initialize(storage,
      UX_HOST_CLASS_STORAGE_DATA_IN,
      UX_HOST_CLASS_STORAGE_READ_CAPACITY_RESPONSE_LENGTH,
      UX_HOST_CLASS_STORAGE_READ_CAPACITY_COMMAND_LENGTH_SBC);

  *(cbw + UX_HOST_CLASS_STORAGE_CBW_CB +
    UX_HOST_CLASS_STORAGE_READ_CAPACITY_OPERATION) =
      UX_HOST_CLASS_STORAGE_SCSI_READ_CAPACITY;

  status = _ux_host_class_storage_transport(storage, response);

  sense = storage->ux_host_class_storage_sense_code;

  if (status == UX_SUCCESS && sense == UX_SUCCESS)
  {
    *p_last_sector = _ux_utility_long_get_big_endian(
        response + UX_HOST_CLASS_STORAGE_READ_CAPACITY_DATA_LBA);
    *p_sector_size = _ux_utility_long_get_big_endian(
        response + UX_HOST_CLASS_STORAGE_READ_CAPACITY_DATA_SECTOR_SIZE);

    /* 校验返回数据合理性 */
    if (*p_last_sector == 0 || *p_sector_size == 0)
    {
      msc_log("[MSC] READ CAPACITY returned invalid data: last_sect=%lu sect_sz=%lu\r\n",
              *p_last_sector, *p_sector_size);
      _ux_utility_memory_free(response);
      return UX_ERROR;
    }
  }
  else
  {
    msc_log("[MSC] READ CAPACITY transport=%u sense=0x%06lX\r\n", status, sense);
    if (status == UX_SUCCESS)
      status = UX_ERROR;  /* sense 错误也作为失败返回 */
  }

  _ux_utility_memory_free(response);
  return status;
}

/**
  * @brief  检测磁盘分区表，获取第一个有效分区的偏移和大小
  * @note   调用前需设置好 msc_active_lun
  * @retval UX_SUCCESS 找到分区; UX_ERROR 无分区表 (全盘使用)
  */
static UINT msc_detect_partition(UX_HOST_CLASS_STORAGE *storage,
                                  ULONG total_sectors, ULONG sector_size)
{
  UCHAR *sector_buf;
  UINT   status;
  ULONG  part_offset;
  ULONG  part_size;

  sector_buf = _ux_utility_memory_allocate(UX_SAFE_ALIGN, UX_CACHE_SAFE_MEMORY, sector_size);
  if (sector_buf == UX_NULL)
  {
    msc_log("[MSC] Partition detect: alloc failed\r\n");
    msc_partition_start = 0;
    msc_partition_size  = total_sectors;
    return UX_MEMORY_INSUFFICIENT;
  }

  /* 读取 sector 0 (MBR) */
  status = _ux_host_class_storage_lock(storage, 5 * TX_TIMER_TICKS_PER_SECOND);
  if (status != UX_SUCCESS)
  {
    _ux_utility_memory_free(sector_buf);
    msc_partition_start = 0;
    msc_partition_size  = total_sectors;
    return UX_ERROR;
  }

  storage->ux_host_class_storage_lun = msc_active_lun;
  storage->ux_host_class_storage_sector_size = sector_size;
  status = _ux_host_class_storage_media_read(storage, 0, 1, sector_buf);
  _ux_host_class_storage_unlock(storage);

  if (status != UX_SUCCESS)
  {
    msc_log("[MSC] Partition detect: read sector 0 failed (0x%02X)\r\n", status);
    _ux_utility_memory_free(sector_buf);
    msc_partition_start = 0;
    msc_partition_size  = total_sectors;
    return UX_ERROR;
  }

  /* 打印 sector 0 关键字节用于诊断 */
  msc_log("[MSC] Sector 0 dump: [0]=0x%02X [1]=0x%02X [2]=0x%02X [510]=0x%02X [511]=0x%02X\r\n",
          sector_buf[0], sector_buf[1], sector_buf[2],
          sector_buf[510], sector_buf[511]);
  msc_log("[MSC] Sector 0 [446..449]: 0x%02X 0x%02X 0x%02X 0x%02X (1st partition entry)\r\n",
          sector_buf[446], sector_buf[447], sector_buf[448], sector_buf[449]);

  /* 使用 FileX API 解析 MBR 分区表 (partition 0) */
  part_offset = 0;
  part_size   = 0;
  status = _fx_partition_offset_calculate(sector_buf, 0, &part_offset, &part_size);
  msc_log("[MSC] _fx_partition_offset_calculate: status=0x%02X offset=%lu size=%lu\r\n",
          status, part_offset, part_size);

  if (status == FX_SUCCESS && part_offset > 0 && part_size > 0)
  {
    msc_partition_start = part_offset;
    msc_partition_size  = part_size;
    msc_log("[MSC] MBR partition found: start=%lu size=%lu\r\n",
            msc_partition_start, msc_partition_size);
  }
  else
  {
    msc_partition_start = 0;
    msc_partition_size  = total_sectors;
    msc_log("[MSC] No MBR partition table, using entire disk\r\n");
  }

  _ux_utility_memory_free(sector_buf);
  return (msc_partition_start > 0) ? UX_SUCCESS : UX_ERROR;
}

/**
  * @brief  自定义 FileX 驱动 — 通过 USBX Storage 进行底层 I/O
  * @note   用于 fx_media_format / fx_media_open，不依赖 UX_HOST_CLASS_STORAGE_MEDIA
  *         使用 msc_partition_start 作为分区偏移，msc_active_lun 选择 LUN
  */
static void msc_fx_driver(FX_MEDIA *media_ptr)
{
  UX_HOST_CLASS_STORAGE *storage;
  UINT status;

  storage = (UX_HOST_CLASS_STORAGE *)media_ptr->fx_media_driver_info;
  if (storage == UX_NULL)
  {
    media_ptr->fx_media_driver_status = FX_PTR_ERROR;
    return;
  }

  switch (media_ptr->fx_media_driver_request)
  {
    case FX_DRIVER_INIT:
      if (storage->ux_host_class_storage_write_protected_media == UX_TRUE)
        media_ptr->fx_media_driver_write_protect = UX_TRUE;
      media_ptr->fx_media_driver_status = FX_SUCCESS;
      break;

    case FX_DRIVER_UNINIT:
      media_ptr->fx_media_driver_status = FX_SUCCESS;
      break;

    case FX_DRIVER_BOOT_READ:
      _ux_host_semaphore_get(&storage->ux_host_class_storage_semaphore, UX_WAIT_FOREVER);
      storage->ux_host_class_storage_lun = msc_active_lun;
      storage->ux_host_class_storage_sector_size = msc_sector_size;
      status = _ux_host_class_storage_media_read(storage,
                  msc_partition_start, 1,
                  media_ptr->fx_media_driver_buffer);
      _ux_host_semaphore_put(&storage->ux_host_class_storage_semaphore);
      if (status == UX_SUCCESS)
      {
        UCHAR *b = media_ptr->fx_media_driver_buffer;
        msc_log("[DRV] BOOT_READ ok: [0]=0x%02X [1]=0x%02X [2]=0x%02X [510]=0x%02X [511]=0x%02X\r\n",
                b[0], b[1], b[2], b[510], b[511]);
      }
      else
      {
        msc_log("[DRV] BOOT_READ failed: 0x%02X\r\n", status);
      }
      media_ptr->fx_media_driver_status = (status == UX_SUCCESS) ? FX_SUCCESS : FX_IO_ERROR;
      break;

    case FX_DRIVER_BOOT_WRITE:
      _ux_host_semaphore_get(&storage->ux_host_class_storage_semaphore, UX_WAIT_FOREVER);
      storage->ux_host_class_storage_lun = msc_active_lun;
      storage->ux_host_class_storage_sector_size = msc_sector_size;
      status = _ux_host_class_storage_media_write(storage,
                  msc_partition_start, 1,
                  media_ptr->fx_media_driver_buffer);
      _ux_host_semaphore_put(&storage->ux_host_class_storage_semaphore);
      media_ptr->fx_media_driver_status = (status == UX_SUCCESS) ? FX_SUCCESS : FX_IO_ERROR;
      break;

    case FX_DRIVER_READ:
      _ux_host_semaphore_get(&storage->ux_host_class_storage_semaphore, UX_WAIT_FOREVER);
      storage->ux_host_class_storage_lun = msc_active_lun;
      storage->ux_host_class_storage_sector_size = msc_sector_size;
      status = _ux_host_class_storage_media_read(storage,
                  media_ptr->fx_media_driver_logical_sector + msc_partition_start,
                  media_ptr->fx_media_driver_sectors,
                  media_ptr->fx_media_driver_buffer);
      _ux_host_semaphore_put(&storage->ux_host_class_storage_semaphore);
      media_ptr->fx_media_driver_status = (status == UX_SUCCESS) ? FX_SUCCESS : FX_IO_ERROR;
      break;

    case FX_DRIVER_WRITE:
      _ux_host_semaphore_get(&storage->ux_host_class_storage_semaphore, UX_WAIT_FOREVER);
      storage->ux_host_class_storage_lun = msc_active_lun;
      storage->ux_host_class_storage_sector_size = msc_sector_size;
      status = _ux_host_class_storage_media_write(storage,
                  media_ptr->fx_media_driver_logical_sector + msc_partition_start,
                  media_ptr->fx_media_driver_sectors,
                  media_ptr->fx_media_driver_buffer);
      _ux_host_semaphore_put(&storage->ux_host_class_storage_semaphore);
      media_ptr->fx_media_driver_status = (status == UX_SUCCESS) ? FX_SUCCESS : FX_IO_ERROR;
      break;

    case FX_DRIVER_FLUSH:
      media_ptr->fx_media_driver_status = FX_SUCCESS;
      break;

    case FX_DRIVER_ABORT:
      media_ptr->fx_media_driver_status = FX_SUCCESS;
      break;

    default:
      media_ptr->fx_media_driver_status = FX_IO_ERROR;
      break;
  }
}

/**
  * @brief  打印 FX_MEDIA 文件系统详情
  */
static void msc_print_media_info(FX_MEDIA *media)
{
  CHAR volume_name[32];
  UINT status;
  ULONG cluster_bytes, total_mb, used_clusters, used_mb, free_mb;

  status = fx_media_volume_get(media, volume_name, FX_BOOT_SECTOR);
  if (status == FX_SUCCESS && volume_name[0] != '\0')
    msc_log("  Volume       : %s\r\n", volume_name);
  else
    msc_log("  Volume       : (none)\r\n");

  msc_log("  Bytes/Sector : %lu\r\n", media->fx_media_bytes_per_sector);
  msc_log("  Sec/Cluster  : %lu\r\n", media->fx_media_sectors_per_cluster);
  msc_log("  Total Sectors: %lu\r\n", media->fx_media_total_sectors);
  msc_log("  Total Clust  : %lu\r\n", media->fx_media_total_clusters);
  msc_log("  Avail Clust  : %lu\r\n", media->fx_media_available_clusters);
  msc_log("  FATs         : %u\r\n",  media->fx_media_number_of_FATs);
  msc_log("  Hidden Sec   : %lu\r\n", media->fx_media_hidden_sectors);

  /* 识别 FAT 类型 */
  if (media->fx_media_12_bit_FAT)
    msc_log("  FAT Type     : FAT12\r\n");
  else if (media->fx_media_32_bit_FAT)
    msc_log("  FAT Type     : FAT32\r\n");
  else
    msc_log("  FAT Type     : FAT16\r\n");

  cluster_bytes = (ULONG)media->fx_media_bytes_per_sector *
                  media->fx_media_sectors_per_cluster;
  total_mb = (ULONG)((unsigned long long)media->fx_media_total_clusters *
             cluster_bytes / (1024UL * 1024UL));
  used_clusters = media->fx_media_total_clusters -
                  media->fx_media_available_clusters;
  used_mb = (ULONG)((unsigned long long)used_clusters * cluster_bytes /
            (1024UL * 1024UL));
  free_mb = total_mb - used_mb;

  msc_log("  Total Space  : %lu MB\r\n", total_mb);
  msc_log("  Used Space   : %lu MB\r\n", used_mb);
  msc_log("  Free Space   : %lu MB\r\n", free_mb);
}

/**
  * @brief  根据磁盘大小选择每簇扇区数 (参考 Microsoft FAT32 规范)
  */
static UINT msc_select_cluster_size(ULONG total_sectors, ULONG sector_size)
{
  unsigned long long disk_bytes = (unsigned long long)total_sectors * sector_size;

  if (disk_bytes <= 256ULL * 1024 * 1024)
    return 1;   /* <= 256MB */
  if (disk_bytes <= 8ULL * 1024 * 1024 * 1024)
    return 8;   /* <= 8GB, 4KB cluster */
  if (disk_bytes <= 16ULL * 1024 * 1024 * 1024)
    return 16;  /* <= 16GB, 8KB cluster */
  if (disk_bytes <= 32ULL * 1024 * 1024 * 1024)
    return 32;  /* <= 32GB, 16KB cluster */
  return 64;    /* > 32GB, 32KB cluster */
}

/* USER CODE END 0 */

/* USER CODE BEGIN 1 */

/**
  * @brief  MSC 主处理线程
  *         - 检测 U 盘插入后打印容量等详细信息
  *         - 检测文件系统，若无有效文件系统则使用 FileX 格式化为 FAT32
  * @param  arg  未使用
  */
void msc_process_thread_entry(ULONG arg)
{
  ULONG actual_flags;
  UINT  status;
  ULONG last_sector;
  ULONG sector_size;
  UINT  capacity_ok;
  UINT  retry;
  UINT  lun;
  UINT  found_lun;
  ULONG total_sectors;
  ULONG capacity_mb;
  ULONG capacity_gb_int;
  ULONG capacity_gb_dec;
  UX_HOST_CLASS_STORAGE_MEDIA *storage_media;
  UINT  media_mounted;
  UX_HOST_CLASS *class_inst;
  UINT  i;
  UINT  sec_per_cluster;
  UX_HOST_CLASS_STORAGE *storage;
  UX_HOST_CLASS_STORAGE_MEDIA *sm;
  FX_MEDIA *m;

  (void)arg;

  while (1)
  {
    /* ======== 等待 MSC 设备连接 ======== */
    tx_event_flags_get(&msc_event_flags, MSC_FLAG_CONNECTED,
                       TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);

    storage = msc_storage_instance;
    if (storage == UX_NULL)
      continue;

    /* 等待 USBX 存储类完成枚举、TEST UNIT READY 和自动挂载
       USBX 存储线程每 2 秒唤醒一次，给足时间让它完成 */
    tx_thread_sleep(3 * TX_TIMER_TICKS_PER_SECOND);

    /* 再次确认设备仍在 */
    storage = msc_storage_instance;
    if (storage == UX_NULL)
    {
      msc_log("[MSC] Device removed during init\r\n");
      continue;
    }

    msc_log("\r\n======================================\r\n");
    msc_log("    USB Mass Storage Device Connected\r\n");
    msc_log("======================================\r\n\r\n");

    /* 打印 storage instance 基本信息 */
    msc_log("[MSC] Storage Instance Info:\r\n");
    msc_log("  State        : %u\r\n", storage->ux_host_class_storage_state);
    msc_log("  Media Type   : %u\r\n", storage->ux_host_class_storage_media_type);
    msc_log("  Sector Size  : %lu (from USBX)\r\n", storage->ux_host_class_storage_sector_size);
    msc_log("  Max LUN      : %u\r\n", storage->ux_host_class_storage_max_lun);
    msc_log("  Write Protect: %s\r\n",
            storage->ux_host_class_storage_write_protected_media ? "Yes" : "No");

    /* 打印各 LUN 的类型信息 */
    for (lun = 0; lun <= storage->ux_host_class_storage_max_lun; lun++)
    {
      msc_log("  LUN[%u] type  : %u  removable: %u\r\n",
              lun,
              storage->ux_host_class_storage_lun_types[lun],
              storage->ux_host_class_storage_lun_removable_media_flags[lun]);
    }

    /* ======== 1. 遍历所有 LUN，通过 TEST UNIT READY + READ CAPACITY 找到有效介质 ======== */
    last_sector = 0;
    sector_size = 0;
    capacity_ok = 0;
    found_lun = 0;

    for (retry = 0; retry < 8 && !capacity_ok; retry++)
    {
      if (retry > 0)
      {
        msc_log("[MSC] Retry %u/8, waiting...\r\n", retry);
        tx_thread_sleep(2 * TX_TIMER_TICKS_PER_SECOND);
      }

      storage = msc_storage_instance;
      if (storage == UX_NULL)
        break;

      status = _ux_host_class_storage_lock(storage, 5 * TX_TIMER_TICKS_PER_SECOND);
      if (status != UX_SUCCESS)
      {
        msc_log("[MSC] Lock failed: 0x%02X\r\n", status);
        continue;
      }

      /* 遍历所有 LUN */
      for (lun = 0; lun <= storage->ux_host_class_storage_max_lun; lun++)
      {
        /* 设置当前 LUN */
        storage->ux_host_class_storage_lun = lun;

        /* TEST UNIT READY */
        status = _ux_host_class_storage_unit_ready_test(storage);
        msc_log("[MSC] LUN %u TEST_UNIT_READY: status=%u sense=0x%06lX\r\n",
                lun, status, storage->ux_host_class_storage_sense_code);

        if (status != UX_SUCCESS || storage->ux_host_class_storage_sense_code != 0)
          continue;

        /* READ CAPACITY */
        status = msc_scsi_read_capacity(storage, &last_sector, &sector_size);
        msc_log("[MSC] LUN %u READ_CAPACITY: status=%u last_sec=%lu sec_sz=%lu\r\n",
                lun, status, last_sector, sector_size);

        if (status == UX_SUCCESS && last_sector > 0 && sector_size > 0)
        {
          capacity_ok = 1;
          found_lun = lun;
          break;
        }
      }

      _ux_host_class_storage_unlock(storage);
    }

    /* ======== 2. 打印容量信息 ======== */
    if (capacity_ok)
    {
      total_sectors   = last_sector + 1;
      capacity_mb     = (ULONG)((unsigned long long)total_sectors * sector_size /
                        (1024UL * 1024UL));
      capacity_gb_int = capacity_mb / 1024;
      capacity_gb_dec = (capacity_mb % 1024) * 100 / 1024;

      msc_log("\r\n[MSC] Disk Capacity (LUN %u):\r\n", found_lun);
      msc_log("  Sector Size  : %lu bytes\r\n", sector_size);
      msc_log("  Total Sectors: %lu\r\n", total_sectors);
      msc_log("  Capacity     : %lu MB (%lu.%02lu GB)\r\n",
              capacity_mb, capacity_gb_int, capacity_gb_dec);
    }
    else
    {
      msc_log("\r\n[MSC] Cannot read disk capacity after all retries.\r\n");
      msc_log("  Possible: card reader with no card, or device not ready.\r\n");
    }

    /* ======== 3. 检查 USBX 是否已自动挂载 FileX 媒体 ======== */
    storage_media = UX_NULL;
    media_mounted = 0;

    storage = msc_storage_instance;
    if (storage != UX_NULL)
    {
      class_inst = storage->ux_host_class_storage_class;
      if (class_inst != UX_NULL && class_inst->ux_host_class_media != UX_NULL)
      {
        sm = (UX_HOST_CLASS_STORAGE_MEDIA *)class_inst->ux_host_class_media;
        for (i = 0; i < UX_HOST_CLASS_STORAGE_MAX_MEDIA; i++, sm++)
        {
          m = &sm->ux_host_class_storage_media;
          msc_log("[MSC] Media slot[%u]: status=%lu lun=%lu id=0x%lX\r\n",
                  i,
                  sm->ux_host_class_storage_media_status,
                  sm->ux_host_class_storage_media_lun,
                  ux_media_id_get(m));
          if (sm->ux_host_class_storage_media_status ==
              UX_HOST_CLASS_STORAGE_MEDIA_MOUNTED &&
              ux_media_id_get(m) != 0)
          {
            storage_media = sm;
            break;
          }
        }
      }
    }

    if (storage_media != UX_NULL)
    {
      media_mounted = 1;
      msc_log("\r\n[MSC] File system detected (auto-mounted by USBX):\r\n");
      msc_print_media_info(&storage_media->ux_host_class_storage_media);
    }
    else if (capacity_ok && storage != UX_NULL)
    {
      /* ======== 4. 尝试用自定义驱动 open，检测是否有文件系统 ======== */
      msc_log("\r\n[MSC] USBX did not auto-mount, trying manual open...\r\n");

      /* 设置活动 LUN 和 sector_size (驱动回调中使用) */
      msc_active_lun = found_lun;
      msc_sector_size = sector_size;
      storage->ux_host_class_storage_sector_size = sector_size;

      /* 检测 MBR 分区表 */
      total_sectors = last_sector + 1;
      msc_detect_partition(storage, total_sectors, sector_size);

      status = fx_media_open(&msc_media, "USB_DISK",
                             msc_fx_driver,
                             (VOID *)storage,
                             msc_media_buffer,
                             MSC_MEDIA_BUF_SIZE);
      if (status == FX_SUCCESS)
      {
        media_mounted = 1;
        msc_log("[MSC] File system detected (manual open):\r\n");
        msc_print_media_info(&msc_media);
        fx_media_close(&msc_media);
      }
      else
      {
        /* ======== 5. 没有有效文件系统 → 格式化为 FAT32 ======== */
        msc_log("[MSC] No valid file system (open err=0x%02X)\r\n", status);
        msc_log("[MSC] Formatting as FAT32, please wait...\r\n");

        sec_per_cluster = msc_select_cluster_size(msc_partition_size, sector_size);

        msc_log("  partition_start = %lu\r\n", msc_partition_start);
        msc_log("  partition_size  = %lu\r\n", msc_partition_size);
        msc_log("  sector_size     = %lu\r\n", sector_size);
        msc_log("  sec_per_cluster = %u\r\n",  sec_per_cluster);

        status = fx_media_format(&msc_media,
                                 msc_fx_driver,
                                 (VOID *)storage,
                                 msc_media_buffer,
                                 MSC_MEDIA_BUF_SIZE,
                                 "USB_DISK",
                                 2,                    /* FAT 副本数 */
                                 0,                    /* 根目录项 (0=FAT32) */
                                 msc_partition_start,  /* hidden sectors */
                                 msc_partition_size,   /* 分区内扇区数 */
                                 sector_size,
                                 sec_per_cluster,
                                 1,                    /* heads */
                                 1);                   /* sectors/track */

        if (status != FX_SUCCESS)
        {
          msc_log("[MSC] Format FAILED: 0x%02X\r\n", status);
        }
        else
        {
          msc_log("[MSC] Format completed successfully!\r\n");

          status = fx_media_open(&msc_media, "USB_DISK",
                                 msc_fx_driver,
                                 (VOID *)storage,
                                 msc_media_buffer,
                                 MSC_MEDIA_BUF_SIZE);
          if (status == FX_SUCCESS)
          {
            media_mounted = 1;
            msc_log("\r\n[MSC] File system info (after formatting):\r\n");
            msc_print_media_info(&msc_media);
            fx_media_close(&msc_media);
          }
          else
          {
            msc_log("[MSC] Open after format FAILED: 0x%02X\r\n", status);
          }
        }
      }
    }
    else
    {
      msc_log("[MSC] No media available, nothing to do.\r\n");
    }

    msc_log("\r\n======================================\r\n");
    msc_log("    MSC Processing %s\r\n",
            media_mounted ? "Complete" : "Failed");
    msc_log("======================================\r\n\r\n");

    /* ======== 等待设备断开 ======== */
    tx_event_flags_get(&msc_event_flags, MSC_FLAG_DISCONNECTED,
                       TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
    msc_log("\r\n[MSC] USB Mass Storage Device Disconnected\r\n\r\n");
  }
}

/* USER CODE END 1 */
