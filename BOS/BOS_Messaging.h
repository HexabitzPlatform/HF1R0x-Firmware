#pragma once

#include "string"
#include "cstdint"
#include "vector"
#include "BOS.h"
#include "BOS_MessageCodes.h"
#include "UARTParser.h"
#include "Porting.h"

// class Messaging
// {
// public:
//     void SendMessagetoModule();
// };

namespace Messaging
{
    BOSStatus SendMessagetoModule(uint8_t dstID, BOSMessageCode code, const std::vector<uint8_t> &params);
};
