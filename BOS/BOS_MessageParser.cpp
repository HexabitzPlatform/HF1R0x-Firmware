// BOS_MessageParser.cpp
#include "BOS_MessageParser.h"
#include <iostream>

void BOS_MessageParser::parseMessage(const std::vector<uint8_t> &payload)
{
    if (payload.size() < 4)
    {
        std::cerr << "Invalid payload: too short\n";
        return;
    }

    // Extract fields
    uint8_t source = payload[1];
    uint8_t code = payload[3];

    std::vector<uint8_t> params;
    if (payload.size() > 4)
    {
        params.insert(params.end(), payload.begin() + 4, payload.end());
    }

    // Route based on message code
    switch (code)
    {
    case 0x01:
        handleCode01(source, params);
        break;
    case 0x02:
        handleCode02(source, params);
        break;
    case 0xFF:
        handleCodeFF(source, params);
        break;
    default:
        std::cerr << "Unknown message code: 0x" << std::hex << static_cast<int>(code) << "\n";
        break;
    }
}

void BOS_MessageParser::handleCode01(uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0x01] Received from module " << static_cast<int>(source)
              << " with " << params.size() << " bytes of parameters\n";
    // Interpret params accordingly
}

void BOS_MessageParser::handleCode02(uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0x02] Execute special task from module " << static_cast<int>(source) << "\n";
    // Handle as needed
}

void BOS_MessageParser::handleCodeFF(uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic
}
