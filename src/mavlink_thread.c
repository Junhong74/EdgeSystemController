/**
 * @file mavlink_thread.c
 * @brief MAVLink communication thread implementation
 * @details Handles telemetry reception, command processing, and AI result forwarding
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "system_config.h"
#include "common_types.h"
#include "ipc_config.h"
#include "mavlink_protocol.h"


#define LOG_LEVEL CONFIG_MAVLINK_THREAD_LOG_LEVEL
LOG_MODULE_REGISTER(mavlink_thread);

/* UART Device */
#define UART_DEVICE_NODE DT_NODELABEL(usart2)

/* Ring buffer for UART RX */
#define UART_RX_RING_BUF_SIZE 512
RING_BUF_DECLARE(uart_rx_ring_buf, UART_RX_RING_BUF_SIZE);

/* MAVLink Parser */
static mavlink_parser_t mavlink_parser;

/* UART Device Handle */
static const struct device *uart_dev;

/* Heartbeat timing */
#define HEARTBEAT_PERIOD_MS 1000
static uint32_t last_heartbeat_time = 0;

/**
 * @brief UART Interrupt Callback
 * @details Called when UART receives data or TX is ready
 */
static void uart_isr_callback(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);
    
    if (!uart_irq_update(dev)) {
        return;
    }
    
    /* Handle RX data */
    if (uart_irq_rx_ready(dev)) {
        uint8_t byte;
        int rx_len;
        
        while ((rx_len = uart_fifo_read(dev, &byte, 1)) > 0) {
            /* Put byte into ring buffer */
            ring_buf_put(&uart_rx_ring_buf, &byte, 1);
        }
    }
}
/**
 * @brief Process received MAVLink command
 * @param msg Received MAVLink message
 * @return Result code for ACK
 */
static uint8_t process_mavlink_command(const mavlink_message_t *msg)
{
    if (msg->msgid != MAVLINK_MSG_ID_COMMAND_LONG) {
        return MAV_RESULT_UNSUPPORTED;
    }
    
    mavlink_command_long_t cmd;
    memcpy(&cmd, msg->payload, sizeof(mavlink_command_long_t));
    
    /* Check if command is for this system */
    if (cmd.target_system != MAVLINK_SYSID_CONTROLLER &&
        cmd.target_system != 0) {  /* 0 = broadcast */
        return MAV_RESULT_DENIED;
    }
    
    sys_command_t sys_cmd;
    sys_cmd.param = 0;
    uint8_t result = MAV_RESULT_ACCEPTED;
    
    LOG_INF("[MAVLink] Received command %u from sys=%u comp=%u\n",
           cmd.command, msg->sysid, msg->compid);
    
    /* Map MAVLink command to system command */
    switch (cmd.command) {
        case MAV_CMD_USER_1:  /* System START */
            sys_cmd.command = CMD_START;
            LOG_INF("[MAVLink] Command: START system");
            break;
            
        case MAV_CMD_USER_2:  /* System STOP */
            sys_cmd.command = CMD_STOP;
            LOG_INF("[MAVLink] Command: STOP system");
            break;
            
        case MAV_CMD_USER_3:  /* Stream START */
            sys_cmd.command = CMD_STREAM_START;
            LOG_INF("[MAVLink] Command: START video streaming");
            break;
            
        case MAV_CMD_USER_4:  /* Stream STOP */
            sys_cmd.command = CMD_STREAM_STOP;
            LOG_INF("[MAVLink] Command: STOP video streaming");
            break;
            
        case MAV_CMD_USER_5:  /* System SHUTDOWN */
            sys_cmd.command = CMD_SHUTDOWN;
            LOG_INF("[MAVLink] Command: SHUTDOWN system");
            break;
            
        default:
            LOG_INF("[MAVLink] Unsupported command: %u", cmd.command);
            result = MAV_RESULT_UNSUPPORTED;
            return result;
    }
    
    /* Put command into system command queue */
    if (k_msgq_put(&sys_cmd_queue, &sys_cmd, K_NO_WAIT) != 0) {
        LOG_ERR("[MAVLink] ERROR: System command queue full!");
        result = MAV_RESULT_TEMPORARILY_REJECTED;
    } else {
        LOG_INF("[MAVLink] Command queued successfully");
    }
    
    return result;
}


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
    mavlink_message_t msg;
    uint8_t byte;
    
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("[MAVLink] Thread started\n");

    
    /* Get UART device */
    uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("[MAVLink] ERROR: UART device not ready!");
        return;
    }
    
    LOG_INF("[MAVLink] UART2 device ready");

    /* Initialize MAVLink parser */
    mavlink_parser_init(&mavlink_parser);
    
    /* Configure UART interrupt */
    uart_irq_callback_user_data_set(uart_dev, uart_isr_callback, NULL);
    uart_irq_rx_enable(uart_dev);
    
    LOG_INF("[MAVLink] UART2 interrupt enabled");
    
    /* Send initial heartbeat */
    mavlink_send_heartbeat(uart_dev);
    mavlink_send_statustext(uart_dev, "System Controller Online", 6);
    last_heartbeat_time = k_uptime_get_32();   
    
    while (1) {
        /* Update heartbeat */
        atomic_inc(&g_health.mavlink_heartbeat);
        
        /* Check for system commands from drone controller */
        /* Process received bytes from ring buffer */
        while (ring_buf_get(&uart_rx_ring_buf, &byte, 1) > 0) {
            if (mavlink_parse_byte(&mavlink_parser, byte, &msg)) {
                /* Complete message received */
                LOG_INF("[MAVLink] Received message: id=%u, sys=%u, comp=%u, len=%u",
                       msg.msgid, msg.sysid, msg.compid, msg.len);
                
                uint8_t result = MAV_RESULT_UNSUPPORTED;
                
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT:
                        LOG_INF("[MAVLink] Heartbeat from sys=%u comp=%u",
                               msg.sysid, msg.compid);
                        /* No ACK needed for heartbeat */
                        break;
                        
                    case MAVLINK_MSG_ID_COMMAND_LONG:
                        /* Process command and send ACK */
                        result = process_mavlink_command(&msg);
                        
                        /* Extract command ID for ACK */
                        mavlink_command_long_t cmd;
                        memcpy(&cmd, msg.payload, sizeof(mavlink_command_long_t));
                        
                        /* Send ACK */
                        mavlink_send_command_ack(uart_dev, cmd.command, result,
                                                 msg.sysid, msg.compid);
                        
                        const char *result_str[] = {
                            "ACCEPTED", "TEMP_REJECTED", "DENIED", 
                            "UNSUPPORTED", "FAILED", "IN_PROGRESS"
                        };
                        LOG_INF("[MAVLink] Sent ACK: cmd=%u, result=%s",
                               cmd.command, result_str[result]);
                        break;
                        
                    default:
                        LOG_WRN("[MAVLink] Unhandled message ID: %u", msg.msgid);
                        break;
                }
            }
        }

        /* Send periodic heartbeat (1Hz) to drone/robotic controller */
        uint32_t current_time = k_uptime_get_32();
        if (current_time - last_heartbeat_time >= HEARTBEAT_PERIOD_MS) {
            mavlink_send_heartbeat(uart_dev);
            last_heartbeat_time = current_time;
            LOG_INF("[MAVLink] Heartbeat sent");
        } 
        
        k_msleep(MAVLINK_UPDATE_RATE_MS);
    }
}