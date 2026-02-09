/**
 * @file common_types.h
 * @brief Common data structures used across threads
 */

#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/sys/atomic.h>
#include "system_config.h"

/**
 * COMMAND MESSAGES
 */

/* System Command Types */
typedef enum {
    CMD_STOP,
    CMD_START,
    CMD_STREAM_START,
    CMD_STREAM_STOP,
    CMD_SHUTDOWN,
    CMD_ERROR
} system_cmd_t;

/* System Command Structure */
typedef struct {
    system_cmd_t command;   /* Command type */
    uint32_t param;         /* Command parameter */
} sys_command_t;

/* Telemetry Data Structure */
typedef struct {
    float lat;          /* Latitude in degrees */
    float lon;          /* Longitude in degrees */
    float alt;          /* Altitude in meters */
    float roll;         /* Roll angle in degrees */
    float pitch;        /* Pitch angle in degrees */
    float yaw;          /* Yaw angle in degrees */
    uint32_t timestamp; /* System timestamp in ms */
} telemetry_data_t;


/* Thread Heartbeat Structure */
typedef struct {
    atomic_t mavlink_heartbeat;
    atomic_t sys_mgr_heartbeat;
} thread_health_t;

/* Thread IDs for health monitoring */
#define THREAD_SYS_MGR  0
#define THREAD_MAVLINK  1
#define THREAD_AI       2
#define THREAD_GCS      3
#define THREAD_HEALTH   4
#define NUM_THREADS     5

#endif /* COMMON_TYPES_H_ */