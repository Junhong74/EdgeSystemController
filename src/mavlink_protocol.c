/**
 * @file mavlink_protocol.c
 * @brief MAVLink protocol implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include "mavlink_protocol.h"

/* MAVLink CRC Extra values (per message ID) */
static const uint8_t mavlink_crc_extra[] = {
    [MAVLINK_MSG_ID_HEARTBEAT] = 50,
    [MAVLINK_MSG_ID_COMMAND_LONG] = 152,
    [MAVLINK_MSG_ID_COMMAND_ACK] = 143,
    [MAVLINK_MSG_ID_STATUSTEXT] = 83,
};

/* X.25 CRC calculation */
static const uint16_t crc_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief Initialize MAVLink parser
 */
void mavlink_parser_init(mavlink_parser_t *parser)
{
    memset(parser, 0, sizeof(mavlink_parser_t));
    parser->state = MAVLINK_PARSE_STATE_IDLE;
    parser->seq = 0;
}

/**
 * @brief Accumulate CRC byte
 */
void mavlink_crc_accumulate(uint8_t data, uint16_t *crc)
{
    uint8_t tmp = data ^ (*crc & 0xff);
    *crc = (*crc >> 8) ^ crc_table[tmp];
}

/**
 * @brief Calculate CRC for buffer
 */
uint16_t mavlink_crc_calculate(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        mavlink_crc_accumulate(buf[i], &crc);
    }
    return crc;
}

/**
 * @brief Finalize CRC with message-specific extra byte
 */
uint16_t mavlink_finalize_crc(uint16_t crc, uint8_t msgid)
{
    if (msgid < sizeof(mavlink_crc_extra)) {
        mavlink_crc_accumulate(mavlink_crc_extra[msgid], &crc);
    }
    return crc;
}

/**
 * @brief Parse single MAVLink byte
 * @return true if complete message received
 */
bool mavlink_parse_byte(mavlink_parser_t *parser, uint8_t byte, 
                        mavlink_message_t *msg_out)
{
    bool msg_received = false;

    switch (parser->state) {
        case MAVLINK_PARSE_STATE_IDLE:
            if (byte == MAVLINK_STX) {
                parser->state = MAVLINK_PARSE_STATE_GOT_STX;
                parser->msg.magic = byte;
                parser->crc = 0xFFFF;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_STX:
            parser->msg.len = byte;
            parser->payload_idx = 0;
            mavlink_crc_accumulate(byte, &parser->crc);
            parser->state = MAVLINK_PARSE_STATE_GOT_LENGTH;
            break;

        case MAVLINK_PARSE_STATE_GOT_LENGTH:
            parser->msg.seq = byte;
            mavlink_crc_accumulate(byte, &parser->crc);
            parser->state = MAVLINK_PARSE_STATE_GOT_SEQ;
            break;

        case MAVLINK_PARSE_STATE_GOT_SEQ:
            parser->msg.sysid = byte;
            mavlink_crc_accumulate(byte, &parser->crc);
            parser->state = MAVLINK_PARSE_STATE_GOT_SYSID;
            break;

        case MAVLINK_PARSE_STATE_GOT_SYSID:
            parser->msg.compid = byte;
            mavlink_crc_accumulate(byte, &parser->crc);
            parser->state = MAVLINK_PARSE_STATE_GOT_COMPID;
            break;

        case MAVLINK_PARSE_STATE_GOT_COMPID:
            parser->msg.msgid = byte;
            mavlink_crc_accumulate(byte, &parser->crc);
            if (parser->msg.len == 0) {
                parser->state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            } else {
                parser->state = MAVLINK_PARSE_STATE_GOT_MSGID;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_MSGID:
            parser->msg.payload[parser->payload_idx++] = byte;
            mavlink_crc_accumulate(byte, &parser->crc);
            if (parser->payload_idx >= parser->msg.len) {
                parser->state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
            parser->msg.checksum = byte;  /* Low byte */
            parser->state = MAVLINK_PARSE_STATE_GOT_CRC1;
            break;

        case MAVLINK_PARSE_STATE_GOT_CRC1:
            parser->msg.checksum |= (uint16_t)byte << 8;  /* High byte */
            
            /* Finalize CRC with extra byte */
            uint16_t calc_crc = mavlink_finalize_crc(parser->crc, 
                                                     parser->msg.msgid);
            
            if (calc_crc == parser->msg.checksum) {
                /* Valid message received */
                if (msg_out) {
                    memcpy(msg_out, &parser->msg, sizeof(mavlink_message_t));
                }
                msg_received = true;
            }
            
            /* Reset parser for next message */
            parser->state = MAVLINK_PARSE_STATE_IDLE;
            break;
    }

    return msg_received;
}

/**
 * @brief Send MAVLink HEARTBEAT message
 */
int mavlink_send_heartbeat(const struct device *uart)
{
    uint8_t buf[MAVLINK_FRAME_MAX_SIZE];
    mavlink_heartbeat_t hb = {
        .custom_mode = 0,
        .type = 6,  /* MAV_TYPE_GCS */
        .autopilot = 8,  /* MAV_AUTOPILOT_INVALID */
        .base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        .system_status = MAV_STATE_ACTIVE,
        .mavlink_version = 3
    };

    static uint8_t seq = 0;
    
    buf[0] = MAVLINK_STX;
    buf[1] = sizeof(mavlink_heartbeat_t);
    buf[2] = seq++;
    buf[3] = MAVLINK_SYSID_CONTROLLER;
    buf[4] = MAVLINK_COMPID_CONTROLLER;
    buf[5] = MAVLINK_MSG_ID_HEARTBEAT;
    
    memcpy(&buf[6], &hb, sizeof(mavlink_heartbeat_t));
    
    uint16_t crc = mavlink_crc_calculate(&buf[1], sizeof(mavlink_heartbeat_t) + 5);
    crc = mavlink_finalize_crc(crc, MAVLINK_MSG_ID_HEARTBEAT);
    
    buf[6 + sizeof(mavlink_heartbeat_t)] = crc & 0xFF;
    buf[7 + sizeof(mavlink_heartbeat_t)] = (crc >> 8) & 0xFF;
    
    int len = 8 + sizeof(mavlink_heartbeat_t);
    
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart, buf[i]);
    }
    
    return len;
}

/**
 * @brief Send MAVLink COMMAND_ACK message
 */
int mavlink_send_command_ack(const struct device *uart, uint16_t command, 
                             uint8_t result, uint8_t target_sys, 
                             uint8_t target_comp)
{
    uint8_t buf[MAVLINK_FRAME_MAX_SIZE];
    mavlink_command_ack_t ack = {
        .command = command,
        .result = result,
        .progress = 0,
        .result_param2 = 0,
        .target_system = target_sys,
        .target_component = target_comp
    };

    static uint8_t seq = 0;
    
    buf[0] = MAVLINK_STX;
    buf[1] = sizeof(mavlink_command_ack_t);
    buf[2] = seq++;
    buf[3] = MAVLINK_SYSID_CONTROLLER;
    buf[4] = MAVLINK_COMPID_CONTROLLER;
    buf[5] = MAVLINK_MSG_ID_COMMAND_ACK;
    
    memcpy(&buf[6], &ack, sizeof(mavlink_command_ack_t));
    
    uint16_t crc = mavlink_crc_calculate(&buf[1], sizeof(mavlink_command_ack_t) + 5);
    crc = mavlink_finalize_crc(crc, MAVLINK_MSG_ID_COMMAND_ACK);
    
    buf[6 + sizeof(mavlink_command_ack_t)] = crc & 0xFF;
    buf[7 + sizeof(mavlink_command_ack_t)] = (crc >> 8) & 0xFF;
    
    int len = 8 + sizeof(mavlink_command_ack_t);
    
    for (int i = 0; i < len; i++) {
        uart_poll_out(uart, buf[i]);
    }
    
    return len;
}

/**
 * @brief Send MAVLink STATUSTEXT message
 */
int mavlink_send_statustext(const struct device *uart, const char *text, 
                            uint8_t severity)
{
    uint8_t buf[MAVLINK_FRAME_MAX_SIZE];
    uint8_t payload[54] = {0};  /* severity(1) + text(50) + id(2) + chunk_seq(1) */
    
    payload[0] = severity;
    strncpy((char *)&payload[1], text, 50);
    
    static uint8_t seq = 0;
    
    buf[0] = MAVLINK_STX;
    buf[1] = 54;
    buf[2] = seq++;
    buf[3] = MAVLINK_SYSID_CONTROLLER;
    buf[4] = MAVLINK_COMPID_CONTROLLER;
    buf[5] = MAVLINK_MSG_ID_STATUSTEXT;
    
    memcpy(&buf[6], payload, 54);
    
    uint16_t crc = mavlink_crc_calculate(&buf[1], 54 + 5);
    crc = mavlink_finalize_crc(crc, MAVLINK_MSG_ID_STATUSTEXT);
    
    buf[60] = crc & 0xFF;
    buf[61] = (crc >> 8) & 0xFF;
    
    for (int i = 0; i < 62; i++) {
        uart_poll_out(uart, buf[i]);
    }
    
    return 62;
}
