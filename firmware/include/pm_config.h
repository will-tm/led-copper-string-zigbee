/*
 * Partition Manager configuration for LED Copper String Controller
 * Compatible with flash_map_partition_manager.c
 * Pro Micro nRF52840 only
 */

#ifndef PM_CONFIG_H
#define PM_CONFIG_H

#include <autoconf.h>

/*
 * Pro Micro nRF52840 layout (with MCUboot):
 *   ID 0: 0x000000 - mcuboot
 *   ID 1: 0x00C000 - slot0_partition (primary app)
 *   ID 2: 0x082000 - slot1_partition (secondary/OTA)
 *   ID 3: 0x0E8000 - ZBOSS NVRAM (32KB)
 *   ID 4: 0x0F0000 - ZBOSS Product Config (4KB)
 *   ID 5: 0x0F8000 - Settings Storage (32KB)
 */

#define PM_NUM 6

#define PM_0_LABEL MCUBOOT
#define PM_1_LABEL MCUBOOT_PRIMARY
#define PM_2_LABEL MCUBOOT_SECONDARY
#define PM_3_LABEL ZBOSS_NVRAM
#define PM_4_LABEL ZBOSS_PRODUCT_CONFIG
#define PM_5_LABEL SETTINGS_STORAGE

#define PM_MCUBOOT_ID                   0
#define PM_MCUBOOT_OFFSET               0x0
#define PM_MCUBOOT_SIZE                 0xC000
#define PM_MCUBOOT_DEV                  flash_controller
#define PM_MCUBOOT_DEFAULT_DRIVER_KCONFIG 1

#define PM_MCUBOOT_PRIMARY_ID           1
#define PM_MCUBOOT_PRIMARY_OFFSET       0xC000
#define PM_MCUBOOT_PRIMARY_SIZE         0x76000
#define PM_MCUBOOT_PRIMARY_DEV          flash_controller
#define PM_MCUBOOT_PRIMARY_DEFAULT_DRIVER_KCONFIG 1

#define PM_MCUBOOT_SECONDARY_ID         2
#define PM_MCUBOOT_SECONDARY_OFFSET     0x82000
#define PM_MCUBOOT_SECONDARY_SIZE       0x66000
#define PM_MCUBOOT_SECONDARY_DEV        flash_controller
#define PM_MCUBOOT_SECONDARY_DEFAULT_DRIVER_KCONFIG 1

#define PM_ZBOSS_NVRAM_ID               3
#define PM_ZBOSS_NVRAM_OFFSET           0xE8000
#define PM_ZBOSS_NVRAM_SIZE             0x8000
#define PM_ZBOSS_NVRAM_DEV              flash_controller
#define PM_ZBOSS_NVRAM_DEFAULT_DRIVER_KCONFIG 1

#define PM_ZBOSS_PRODUCT_CONFIG_ID      4
#define PM_ZBOSS_PRODUCT_CONFIG_OFFSET  0xF0000
#define PM_ZBOSS_PRODUCT_CONFIG_SIZE    0x1000
#define PM_ZBOSS_PRODUCT_CONFIG_DEV     flash_controller
#define PM_ZBOSS_PRODUCT_CONFIG_DEFAULT_DRIVER_KCONFIG 1

#define PM_SETTINGS_STORAGE_ID          5
#define PM_SETTINGS_STORAGE_OFFSET      0xF8000
#define PM_SETTINGS_STORAGE_SIZE        0x8000
#define PM_SETTINGS_STORAGE_DEV         flash_controller
#define PM_SETTINGS_STORAGE_DEFAULT_DRIVER_KCONFIG 1

/* Legacy address macros for compatibility */
#define PM_ZBOSS_NVRAM_ADDRESS          PM_ZBOSS_NVRAM_OFFSET
#define PM_ZBOSS_PRODUCT_CONFIG_ADDRESS PM_ZBOSS_PRODUCT_CONFIG_OFFSET

#endif /* PM_CONFIG_H */
