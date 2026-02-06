// State machine includes
//#include <zephyr/smf.h>
#include <zephyr/logging/log.h>

#include "device_sm.h"

#define LOG_LEVEL CONFIG_DEVICE_SM_LOG_LEVEL
LOG_MODULE_REGISTER(device_sm);

static state_t current_state = INIT;
static state_t previous_state = INIT;

void fsm_init(void)
{
    current_state = INIT;
}

state_t fsm_get_current_state(void)
{
    return current_state;
}

state_t fsm_get_previous_state(void)
{
    return previous_state;
}


// State transition function
void fsm_event_handler(event_t event) 
{
    previous_state = current_state;
    
    switch (current_state) {
        case INIT:
            if (event == EVENT_INIT_DONE) {
                current_state = IDLE;
                LOG_INF("Transition: INIT -> IDLE");
            }
            else if (event == EVENT_INIT_FAILED) {
                current_state = FAULT;
                LOG_INF("Transition: INIT -> FAULT");
            }
            break;
        case IDLE:
            if (event == EVENT_CMD_START) {
                current_state = ACTIVE;
                LOG_INF("Transition: IDLE -> ACTIVE");
            } 
            else if (event == EVENT_CMD_SHUTDOWN) {
                current_state = SHUTDOWN;
                LOG_INF("Transition: IDLE -> SHUTDOWN");
            }
            else if (event == EVENT_FAULT_DETECTED) {
                current_state = FAULT;
                LOG_INF("Transition: IDLE -> FAULT");
            }
            break;
        case ACTIVE:
            if (event == EVENT_STREAM_STARTED) {
                current_state = STREAMING;
                LOG_INF("Transition: ACTIVE -> STREAMING");
            } 
            else if (event == EVENT_CMD_STOP) {
                current_state = IDLE;
                LOG_INF("Transition: ACTIVE -> IDLE");
            }
            else if (event == EVENT_CMD_SHUTDOWN) {
                current_state = SHUTDOWN;
                LOG_INF("Transition: ACTIVE -> SHUTDOWN");
            }
            else if (event == EVENT_FAULT_DETECTED) {
                current_state = FAULT;
                LOG_INF("Transition: ACTIVE -> FAULT");
            }
            break;
        case STREAMING:
            if (event == EVENT_STREAM_DEGRADED) {
                current_state = DEGRADED;
                LOG_INF("Transition: STREAMING -> DEGRADED");
            } 
            else if (event == EVENT_STREAM_STOPPED) {
                current_state = ACTIVE;
                LOG_INF("Transition: STREAMING -> ACTIVE");
            }
            else if (event == EVENT_CMD_SHUTDOWN) {
                current_state = SHUTDOWN;
                LOG_INF("Transition: STREAMING -> SHUTDOWN");
            }
            else if (event == EVENT_FAULT_DETECTED) {
                current_state = FAULT;
                LOG_INF("Transition: STREAMING -> FAULT");
            }
            break;
        case DEGRADED:
            if (event == EVENT_STREAM_RECOVERED) {
                current_state = STREAMING;
                LOG_INF("Transition: DEGRADED -> STREAMING");
            } 
            else if (event == EVENT_STREAM_STOPPED) {
                current_state = ACTIVE;
                LOG_INF("Transition: DEGRADED -> ACTIVE");
            }
            else if (event == EVENT_CMD_SHUTDOWN) {
                current_state = SHUTDOWN;
                LOG_INF("Transition: DEGRADED -> SHUTDOWN");
            }
            else if (event == EVENT_FAULT_DETECTED) {
                current_state = FAULT;
                LOG_INF("Transition: DEGRADED -> FAULT");
            }
            break;     
        case FAULT:
            if (event == EVENT_FAULT_RECOVERED) {
                current_state = IDLE;
                LOG_INF("Transition: FAULT -> IDLE");
            }
            break;
        case SHUTDOWN:
            // No transitions out of SHUTDOWN in this simple model
            break;
        default:
            LOG_INF("Unknown state: %d", current_state);
            break;
    }
}