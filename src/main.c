/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <inttypes.h>
#include <zephyr/logging/log.h>

#include "system_config.h"
#include "device_fsm.h"
#include "common_types.h"
#include "ipc_config.h"
#include "threads.h"

#define LOG_LEVEL CONFIG_MAIN_LOG_LEVEL
LOG_MODULE_REGISTER(main);

/*
 * GLOBAL STATE VARIABLES
 */
system_state_t system_state = INIT;
volatile bool system_running = true;

thread_health_t thread_health[NUM_THREADS] = {
    {0, 0, "SysMgr"},
    {0, 0, "MAVLink"},
    {0, 0, "AI"},
    {0, 0, "Video"},
    {0, 0, "GCS"},
    {0, 0, "Health"}
};

/**
 * IPC PRIMITIVES DEFINITIONS
 */

/* Message Queues */
K_MSGQ_DEFINE(sys_cmd_queue, sizeof(sys_command_t), 10, 4);



/**
 * THREAD DEFINITIONS
 */
K_THREAD_DEFINE(sys_mgr_tid, SYS_MGR_STACK_SIZE, 
                sys_mgr_thread, NULL, NULL, NULL,
                SYS_MGR_PRIORITY, 0, 0);


/* UTILITY FUNCTIONS IMPLEMENTATION */
void update_thread_heartbeat(uint8_t thread_id)
{
	if (thread_id < NUM_THREADS) {
		thread_health[thread_id].last_heartbeat = k_uptime_get_32();
		thread_health[thread_id].is_alive = 1;
	}
}


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/**
 * MAIN ENTRY POINT
 */
int main(void)
{
	printk("  Edge AI Vision System Controller\n");
	printk("  Platform: STM32F767ZI\n");
	printk("  Threads: 6 (SysMgr, MAVLink, AI, Video, GCS, Health)\n\n");

	/* All threads are auto-started by K_THREAD_DEFINE */
	/* Main execution continues in main_thread() */

	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}
