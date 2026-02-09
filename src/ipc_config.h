/**
 * @file ipc_config.h
 * @brief Inter-Process Communication configuration
 */

#ifndef IPC_CONFIG_H
#define IPC_CONFIG_H

#include <zephyr/kernel.h>

#include "common_types.h"
#include "system_config.h"
#include "device_fsm.h"

/* MESSAGE QUEUE DECLARATIONS */
extern struct k_msgq mavlink_tx_queue;
extern struct k_msgq sys_cmd_queue;

/* GLOBAL STATE */
extern system_state_t system_state;

/* External Global State */
extern volatile thread_health_t g_health;
extern volatile bool g_system_running;


/* UTILITY FUNCTIONS */
void update_thread_heartbeat(uint8_t thread_id);

#endif /* IPC_CONFIG_H */