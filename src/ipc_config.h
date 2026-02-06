/**
 * @file ipc_config.h
 * @brief Inter-Process Communication configuration
 */

#ifndef IPC_CONFIG_H
#define IPC_CONFIG_H

#include <zephyr/kernel.h>
#include "common_types.h"
#include "system_config.h"

/**
 * MESSAGE QUEUE DECLARATIONS
 */
extern struct k_msgq sys_cmd_queue;

/**
 * GLOBAL STATE
 */
extern system_state_t system_state;
extern volatile bool system_running;
extern thread_health_t thread_health[NUM_THREADS];

/**
 * UTILITY FUNCTIONS
 */
void update_thread_heartbeat(uint8_t thread_id);
void set_system_state(system_state_t new_state);
system_state_t get_system_state(void);


#endif /* IPC_CONFIG_H */