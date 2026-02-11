/**
 * @file system_config.h
 * @brief System-wide configuration and constants
 */

#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

#include <stdint.h>

typedef struct dev_data
{
    uint8_t _reserved;
} dev_data_t;

/**
 * THREAD STACK SIZES
 */
#define SYS_MGR_STACK_SIZE      2048
#define MAVLINK_STACK_SIZE      2048
#define HEALTH_STACK_SIZE       1024

/* Queue Sizes */
#define MAVLINK_TX_QUEUE_SIZE   20
#define AI_RESULT_QUEUE_SIZE    10
#define SYS_CMD_QUEUE_SIZE      10

/* Timing Parameters */
#define SYSMGR_UPDATE_RATE_MS   200   /* 5Hz */
#define MAVLINK_UPDATE_RATE_MS  20    /* 50Hz */
#define HEALTH_CHECK_PERIOD_MS  500   /* 2Hz */

/**
 * THREAD PRIORITIES (Lower number = Higher priority in Zephyr)
 */
#define HEALTH_PRIORITY         1   // Highest - Safety critical
#define MAVLINK_PRIORITY        3   // High - Time-critical drone comms
#define AI_PRIORITY             5   // Medium-high - Inference deadlines
#define GCS_PRIORITY            9   // Medium-low - Network async
#define SYS_MGR_PRIORITY       11   // Lowest - Supervisory only

#endif /* SYSTEM_CONFIG_H_ */