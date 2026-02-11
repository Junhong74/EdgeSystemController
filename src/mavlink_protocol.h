/**
 * @file mavlink_protocol.h
 * @brief MAVLink protocol definitions and message structures
 */

#ifndef MAVLINK_PROTOCOL_H_
#define MAVLINK_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

/* MAVLink Protocol Constants */
#define MAVLINK_STX                 0xFE    /* MAVLink v1 start byte */
#define MAVLINK_STX_V2              0xFD    /* MAVLink v2 start byte */
#define MAVLINK_MAX_PAYLOAD_LEN     255
#define MAVLINK_FRAME_MAX_SIZE      280

/* System and Component IDs */
#define MAVLINK_SYSID_CONTROLLER    1       /* This system */
#define MAVLINK_COMPID_CONTROLLER   1       /* This component */
#define MAVLINK_SYSID_DRONE         2       /* Drone system ID */

/* MAVLink Message IDs (simplified subset) */
#define MAVLINK_MSG_ID_HEARTBEAT            0
#define MAVLINK_MSG_ID_COMMAND_LONG         76
#define MAVLINK_MSG_ID_COMMAND_ACK          77
#define MAVLINK_MSG_ID_STATUSTEXT           253

/* MAV_CMD - Command definitions */
#define MAV_CMD_DO_SET_MODE                 176
#define MAV_CMD_USER_1                      31010  /* System START */
#define MAV_CMD_USER_2                      31011  /* System STOP */
#define MAV_CMD_USER_3                      31012  /* Stream START */
#define MAV_CMD_USER_4                      31013  /* Stream STOP */
#define MAV_CMD_USER_5                      31014  /* System SHUTDOWN */

/* MAV_RESULT - Command result codes */
#define MAV_RESULT_ACCEPTED                 0
#define MAV_RESULT_TEMPORARILY_REJECTED     1
#define MAV_RESULT_DENIED                   2
#define MAV_RESULT_UNSUPPORTED              3
#define MAV_RESULT_FAILED                   4
#define MAV_RESULT_IN_PROGRESS              5

/* MAV_STATE */
#define MAV_STATE_ACTIVE                    4

/* MAV_MODE_FLAG */
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED   1

/* Parser States */
typedef enum {
    MAVLINK_PARSE_STATE_IDLE = 0,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1
} mavlink_parse_state_t;

/* MAVLink Message Structure */
typedef struct {
    uint8_t magic;              /* STX */
    uint8_t len;                /* Payload length */
    uint8_t seq;                /* Sequence number */
    uint8_t sysid;              /* System ID */
    uint8_t compid;             /* Component ID */
    uint8_t msgid;              /* Message ID */
    uint8_t payload[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t checksum;          /* CRC */
} __attribute__((packed)) mavlink_message_t;

/* MAVLink HEARTBEAT Message */
typedef struct {
    uint32_t custom_mode;
    uint8_t type;
    uint8_t autopilot;
    uint8_t base_mode;
    uint8_t system_status;
    uint8_t mavlink_version;
} __attribute__((packed)) mavlink_heartbeat_t;

/* MAVLink COMMAND_LONG Message */
typedef struct {
    float param1;
    float param2;
    float param3;
    float param4;
    float param5;
    float param6;
    float param7;
    uint16_t command;
    uint8_t target_system;
    uint8_t target_component;
    uint8_t confirmation;
} __attribute__((packed)) mavlink_command_long_t;

/* MAVLink COMMAND_ACK Message */
typedef struct {
    uint16_t command;
    uint8_t result;
    uint8_t progress;
    int32_t result_param2;
    uint8_t target_system;
    uint8_t target_component;
} __attribute__((packed)) mavlink_command_ack_t;

/* MAVLink Parser Context */
typedef struct {
    mavlink_parse_state_t state;
    mavlink_message_t msg;
    uint16_t payload_idx;
    uint16_t crc;
    uint8_t seq;                /* Sequence counter for TX */
} mavlink_parser_t;

/* Function Prototypes */
void mavlink_parser_init(mavlink_parser_t *parser);
bool mavlink_parse_byte(mavlink_parser_t *parser, uint8_t byte, 
                        mavlink_message_t *msg_out);
uint16_t mavlink_crc_calculate(const uint8_t *buf, uint16_t len);
void mavlink_crc_accumulate(uint8_t data, uint16_t *crc);
uint16_t mavlink_finalize_crc(uint16_t crc, uint8_t msgid);

int mavlink_send_heartbeat(const struct device *uart);
int mavlink_send_command_ack(const struct device *uart, uint16_t command, 
                             uint8_t result, uint8_t target_sys, 
                             uint8_t target_comp);
int mavlink_send_statustext(const struct device *uart, const char *text, 
                            uint8_t severity);

#endif /* MAVLINK_PROTOCOL_H_ */