#pragma once

#include "string"
#include "cstdint"
#include "vector"

// class Messaging
// {
// public:
//     void SendMessagetoModule();
// };

namespace Messaging
{
    void SendMessagetoModule(uint8_t src, uint8_t dst, uint16_t code, const std::vector<uint8_t> &params);
};
