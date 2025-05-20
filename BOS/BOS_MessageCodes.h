#pragma once

#include <cstdint>

// Enum class to represent BOS message codes clearly and safely
enum class BOSMessageCode : uint16_t
{
    // BOS Message Codes
    CODE_UNKNOWN_MESSAGE = 0,
    CODE_PING = 1,
    CODE_CODE_IND_ON = 3,
    CODE_IND_OFF = 4,
    CODE_IND_TOGGLE = 5,

    CODE_EXPLORE_ADJ = 12,
    CODE_EXPLORE_ADJ_RESPONSE = 13,
    CODE_PORT_DIRECTION = 14,
    COCODE_MODULE_IDDE_2 = 16,
    CODE_TOPOLOGY = 17,

    CODE_READ_REMOTE = 30,
    CODE_READ_REMOTE_RESPONSE = 31,
    CODE_WRITE_REMOTE = 32,
    CODE_WRITE_REMOTE_RESPONSE = 33,

    ENABLE_STOP_MODE_UARTX = 47,

    // CODE_2 = 0xFF,
    // CODE_2 = 0xFF,
    // CODE_2 = 0xFF,
    // CODE_2 = 0xFF,
    // CODE_2 = 0xFF,
    // CODE_2 = 0xFF,

    // H01R0x
    CODE_H01R0_ON = 100,
    CODE_H01R0_OFF = 101,
    CODE_H01R0_TOGGLE = 102,
    CODE_H01R0_COLOR = 103,
    CODE_H01R0_PULSE = 104,
    CODE_H01R0_SWEEP = 105,
    CODE_H01R0_DIM = 106

};
