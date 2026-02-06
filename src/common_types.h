/**
 * @file common_types.h
 * @brief Common data structures used across threads
 */

#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * COMMAND MESSAGES
 */
typedef struct {
    uint8_t cmd_type;
    uint32_t param1;
    uint32_t param2;
    void *data_ptr;
} sys_command_t;

/* Command Types */
#define CMD_START           0x01
#define CMD_STOP            0x02
#define CMD_STREAM_START    0x03
#define CMD_STREAM_STOP     0x04
#define CMD_SHUTDOWN        0x05
#define CMD_ERROR           0xFF


/**
 * THREAD HEALTH
 */
typedef struct {
    uint32_t last_heartbeat;
    uint8_t is_alive;
    char thread_name[16];
} thread_health_t;

/* Thread IDs for health monitoring */
#define THREAD_SYS_MGR  0
#define THREAD_MAVLINK  1
#define THREAD_AI       2
#define THREAD_VIDEO    3
#define THREAD_GCS      4
#define THREAD_HEALTH   5
#define NUM_THREADS     6

#endif /* COMMON_TYPES_H_ */