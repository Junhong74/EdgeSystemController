/**
 * @file sys_mgr_thread.c
 * @brief System manager thread
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include "device_fsm.h"
#include "system_config.h"
#include "common_types.h"
#include "ipc_config.h"

#define LOG_LEVEL CONFIG_SYS_MGR_THREAD_LOG_LEVEL
LOG_MODULE_REGISTER(sys_mgr_thread);


void sys_mgr_thread(void *p1, void *p2, void *p3)
{
    sys_command_t cmd;
    int ret;

    LOG_INF("System Manager Thread started");

    /* System Initialization */
    fsm_init();
    k_sleep(K_MSEC(100)); // Allow other threads to initialize

	fsm_event_handler(EVENT_INIT_DONE);
    LOG_INF("System initialized - entering IDLE state");

    while (system_running) {
        update_thread_heartbeat(THREAD_SYS_MGR);

        /* Process commands from other threads */
        ret = k_msgq_get(&sys_cmd_queue, &cmd, K_MSEC(100));
        if (ret == 0) {
            LOG_INF("Received command: type=0x%02x", cmd.cmd_type);

            system_state_t sys_state = fsm_get_current_state();

            switch (cmd.cmd_type) {
                case CMD_START:
                    if (sys_state == IDLE) {
                        fsm_event_handler(EVENT_CMD_START);
                    }
                    break;

                case CMD_STOP:
                    if (sys_state == ACTIVE) {
                        fsm_event_handler(EVENT_CMD_STOP);
                    }
                    break;

                case CMD_STREAM_START:
                    if (sys_state == ACTIVE) {
                        fsm_event_handler(EVENT_STREAM_STARTED);
                    }
                    break;

                case CMD_STREAM_STOP:
                    if ((sys_state == STREAMING) ||
                        (sys_state == DEGRADED)) {
                        fsm_event_handler(EVENT_STREAM_STOPPED);
                    }
                    break;

                case CMD_SHUTDOWN:
                    if (sys_state != INIT) {
                        fsm_event_handler(EVENT_CMD_SHUTDOWN);
                    }
                    break;

                case CMD_ERROR:
                    fsm_event_handler(EVENT_FAULT_DETECTED);
                    k_sleep(K_MSEC(1000));
                    break;

                default:
                    LOG_INF("Unknown command: 0x%02x", cmd.cmd_type);
                    break;
            }
        }

        k_sleep(K_MSEC(200)); // Low-frequency supervisory loop
    }
}