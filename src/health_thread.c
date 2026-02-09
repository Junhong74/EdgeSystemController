/**
 * @file health_thread.c
 * @brief Health monitoring thread implementation
 * @details Monitors all threads and triggers recovery on failures
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "system_config.h"
#include "common_types.h"
#include "ipc_config.h"

LOG_MODULE_REGISTER(health_thread);
/**
 * @brief Health Monitor Thread Main Loop
 * @details
 * - Checks all thread heartbeats every 500ms
 * - Detects thread hangs/failures
 * - Triggers recovery via system commands
 * 
 * @param p1 Unused parameter
 * @param p2 Unused parameter
 * @param p3 Unused parameter
 */
void health_thread(void *p1, void *p2, void *p3)
{
    uint32_t last_mavlink_heartbeat = 0;
    uint32_t last_sys_mgr_heartbeat = 0;
    uint32_t current_mavlink_heartbeat;
    uint32_t current_sys_mgr_heartbeat;
    sys_command_t error_cmd;
    uint32_t check_count = 0;
    
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("[Health] Thread started");
    LOG_INF("[Health] Monitoring period: %dms", HEALTH_CHECK_PERIOD_MS);
    
    while (1) {
        /* Wait for health check interval */
        k_msleep(HEALTH_CHECK_PERIOD_MS);
        check_count++;
        
        bool all_healthy = true;
        
        /* Atomically read current heartbeat values */
        current_mavlink_heartbeat = atomic_get(&g_health.mavlink_heartbeat);
        current_sys_mgr_heartbeat = atomic_get(&g_health.sys_mgr_heartbeat);
        
        /* Check MAVLink thread */
        if (current_mavlink_heartbeat == last_mavlink_heartbeat) {
            LOG_ERR("[Health] ERROR: MAVLink thread HUNG!");
            all_healthy = false;
        }
        
        /* Check SysMgr thread */
        if (current_sys_mgr_heartbeat == last_sys_mgr_heartbeat) {
            LOG_ERR("[Health] ERROR: SysMgr thread HUNG!");
            all_healthy = false;
        }
        
        if (!all_healthy) {
            /* Trigger error recovery */
            error_cmd.command = CMD_ERROR;
            error_cmd.param = check_count;
            k_msgq_put(&sys_cmd_queue, &error_cmd, K_NO_WAIT);
            
            LOG_INF("[Health] Recovery command sent");

        } else {
            /* Periodic status report (every 10 checks = 5 seconds) */
            if (check_count % 10 == 0) {
                printk("[Health] All threads OK (check #%u)\n", check_count);
                printk("[Health]   MAVLink: %u, SysMgr: %u\n",
                       current_mavlink_heartbeat,
                       current_sys_mgr_heartbeat);
            }
        }
        
        /* Save current state for next comparison */
        last_mavlink_heartbeat = current_mavlink_heartbeat;
        last_sys_mgr_heartbeat = current_sys_mgr_heartbeat;
    }
}
