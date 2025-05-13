#pragma once

#include <cstdint>

// Enum class to represent BOS message codes clearly and safely
enum class BOSMessageCode : uint8_t
{
    CODE_PING = 0x01,
    CODE_1 = 0x02,
    CODE_2 = 0xFF

};
