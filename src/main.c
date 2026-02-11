/**
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <inttypes.h>
#include <zephyr/logging/log.h>

#include "system_config.h"
#include "device_fsm.h"
#include "common_types.h"
#include "ipc_config.h"
#include "thread_interface.h"

#define LOG_LEVEL CONFIG_MAIN_LOG_LEVEL
LOG_MODULE_REGISTER(main);

/*
 * GLOBAL STATE VARIABLES
 */
system_state_t system_state = INIT;
/* Global System State */
thread_health_t g_health = {0};
volatile bool g_system_running = false;


/**
 * IPC PRIMITIVES DEFINITIONS
 */

/* Message Queues */
K_MSGQ_DEFINE(sys_cmd_queue, sizeof(sys_command_t), SYS_CMD_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(mavlink_tx_queue, sizeof(telemetry_data_t), MAVLINK_TX_QUEUE_SIZE, 4);

/**
 * THREAD DEFINITIONS
 */
/* Define Thread Stacks */
K_THREAD_STACK_DEFINE(mavlink_stack, MAVLINK_STACK_SIZE);
K_THREAD_STACK_DEFINE(sys_mgr_stack, SYS_MGR_STACK_SIZE);
K_THREAD_STACK_DEFINE(health_stack, HEALTH_STACK_SIZE);

/* Thread Control Blocks */
static struct k_thread mavlink_thread_data;
static struct k_thread sys_mgr_thread_data;
static struct k_thread health_thread_data;

/* Thread IDs */
static k_tid_t mavlink_tid;
static k_tid_t sys_mgr_tid;
static k_tid_t health_tid;

#define SLEEP_TIME_MS   1000

int main(void)
{
	LOG_INF("  Edge AI Vision System Controller\n");
	LOG_INF("  Platform: STM32F767ZI\n");
	LOG_INF("  Threads: 6 (SysMgr, MAVLink, AI, Video, GCS, Health)\n\n");

	/* All threads are auto-started by K_THREAD_DEFINE */
	/* Create MAVLink Thread */
    mavlink_tid = k_thread_create(&mavlink_thread_data, mavlink_stack,
                                   K_THREAD_STACK_SIZEOF(mavlink_stack),
                                   mavlink_thread,
                                   NULL, NULL, NULL,
                                   MAVLINK_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(mavlink_tid, "mavlink");
    LOG_INF("Created: MAVLink Thread (Priority %d)", MAVLINK_PRIORITY);
    
    /* Create SysMgr Thread */
    sys_mgr_tid = k_thread_create(&sys_mgr_thread_data, sys_mgr_stack,
                                  K_THREAD_STACK_SIZEOF(sys_mgr_stack),
                                  sys_mgr_thread,
                                  NULL, NULL, NULL,
                                  SYS_MGR_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(sys_mgr_tid, "sys_mgr");
    LOG_INF("Created: SysMgr Thread (Priority %d)", SYS_MGR_PRIORITY);
    
    /* Create Health Monitor Thread */
    health_tid = k_thread_create(&health_thread_data, health_stack,
                                  K_THREAD_STACK_SIZEOF(health_stack),
                                  health_thread,
                                  NULL, NULL, NULL,
                                  HEALTH_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(health_tid, "health");
    LOG_INF("Created: Health Thread (Priority %d)", HEALTH_PRIORITY);

	while (1) {
		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}
