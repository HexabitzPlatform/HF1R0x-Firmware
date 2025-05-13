#pragma once

#include <vector>
#include <cstdint>
#include "BOS_MessageCodes.h"

namespace BOS
{
    // Initializes the BOS system (sets up UART receive loop, etc.)
    void initBOS();

    // Handles a full validated BOS message (called internally after UART parsing)
    void handleBOSMessage(const std::vector<uint8_t> &message);

    void processBOSMessage(const std::vector<unsigned char>& message);

}
