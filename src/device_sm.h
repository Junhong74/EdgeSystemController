#ifndef DEVICE_SM_H_
#define DEVICE_SM_H_

/* List of end node states */
typedef enum {
    INIT, 
    IDLE,
    ACTIVE,
    STREAMING,
    DEGRADED,
    SHUTDOWN,
    FAULT
} state_t;

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
} event_t;

void fsm_init(void);
state_t fsm_get_current_state(void);
state_t fsm_get_previous_state(void);
void fsm_event_handler(event_t event);

#endif /* DEVICE_SM_H_ */