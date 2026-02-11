/**
 * @file device_fsm.h
 * @brief Internal device finite state machine definitions
 */

#ifndef DEVICE_FSM_H_
#define DEVICE_FSM_H_

/* List of end node states */
typedef enum {
    INIT, 
    IDLE,
    ACTIVE,
    STREAMING,
    DEGRADED,
    SHUTDOWN,
    FAULT
} system_state_t;

// Define events
typedef enum {
    EVENT_INIT_DONE,
    EVENT_INIT_FAILED,
    EVENT_CMD_START,
    EVENT_CMD_STOP,
    EVENT_CMD_SHUTDOWN,
    EVENT_STREAM_STARTED,
    EVENT_STREAM_STOPPED,
    EVENT_STREAM_DEGRADED,
    EVENT_STREAM_RECOVERED,
    EVENT_FAULT_DETECTED,
    EVENT_FAULT_RECOVERED
} system_event_t;

void fsm_init(void);
system_state_t fsm_get_current_state(void);
system_state_t fsm_get_previous_state(void);
void fsm_event_handler(system_event_t event);

#endif /* DEVICE_FSM_H_ */