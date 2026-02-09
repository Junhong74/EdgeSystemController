/**
 * @file mavlink_thread.c
 * @brief MAVLink communication thread implementation
 * @details Handles telemetry reception, command processing, and AI result forwarding
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/logging/log.h>

#include "system_config.h"
#include "common_types.h"
#include "ipc_config.h"


#define LOG_LEVEL CONFIG_MAVLINK_THREAD_LOG_LEVEL
LOG_MODULE_REGISTER(mavlink_thread);

/**
 * @brief MAVLink Thread Main Loop
 * @details
 * - Receives telemetry from drone controller at 50Hz
 * - Processes system commands from drone
 * - Forwards AI detection results to drone controller
 * 
 * @param p1 Unused parameter
 * @param p2 Unused parameter
 * @param p3 Unused parameter
 */
void mavlink_thread(void *p1, void *p2, void *p3)
{
    telemetry_data_t telem;
    sys_command_t cmd;
    
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("[MAVLink] Thread started\n");
    
    while (1) {
        /* Update heartbeat atomically */
        atomic_inc(&g_health.mavlink_heartbeat);
        
        /* Simulate receiving telemetry from drone controller (50Hz) */
        /* TODO: Replace with actual UART/Serial MAVLink reception */
        telem.lat = 1.3521f;  /* Singapore coordinates */
        telem.lon = 103.8198f;
        telem.alt = 100.0f;
        telem.roll = 0.0f;
        telem.pitch = 0.0f;
        telem.yaw = 90.0f;
        telem.timestamp = k_uptime_get_32();
        
        /* Put telemetry into queue (non-blocking) */
        if (k_msgq_put(&mavlink_tx_queue, &telem, K_NO_WAIT) != 0) {
            /* Queue full - purge old data and retry */
            k_msgq_purge(&mavlink_tx_queue);
            k_msgq_put(&mavlink_tx_queue, &telem, K_NO_WAIT);
        }
        
        /* Check for system commands from drone controller */
        /* TODO: Parse actual MAVLink command messages */
        static uint32_t cmd_counter = 0;
        if (++cmd_counter % 100 == 0) {
            cmd.command = CMD_START;
            cmd.param = 0;
            k_msgq_put(&sys_cmd_queue, &cmd, K_NO_WAIT);
        }

        
        /* 50Hz update rate = 20ms period */
        k_msleep(MAVLINK_UPDATE_RATE_MS);
    }
}
