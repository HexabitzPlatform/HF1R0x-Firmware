#pragma once

#include "string"
#include "cstdint"
#include "vector"

// class Messaging
// {
// public:
//     void SendMessagetoModule();
// };

std::vector<uint8_t> buildBOSPacket(uint8_t dst, uint16_t code, const std::vector<uint8_t> &params, uint8_t src = 1);