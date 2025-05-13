#include "BOS_MessageParser.h"
#include "BOS_MessageCodes.h"
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

    // Convert raw byte to BOSMessageCode enum using static_cast.
    // This is needed because enum class does not implicitly convert from uint8_t,
    // ensuring strong type safety and preventing invalid enum usage.
    // this byte (uint8_t) actually represents a valid BOSMessageCode, so let me treat it that way.
    BOSMessageCode code = static_cast<BOSMessageCode>(payload[3]);

    std::vector<uint8_t> params;

    if (payload.size() > 4)
    {
        params.insert(params.end(), payload.begin() + 4, payload.end());
    }

    // Route based on message code
    switch (code)
    {
    case BOSMessageCode::CODE_PING:
        handleCode01(source, params);
        break;

    case BOSMessageCode::CODE_1:
        handleCode02(source, params);
        break;

    case BOSMessageCode::CODE_2:
        handleCodeFF(source, params);
        break;

    default:
        std::cerr << "Unknown message code: 0x" << std::hex << static_cast<int>(payload[3]) << "\n";
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
