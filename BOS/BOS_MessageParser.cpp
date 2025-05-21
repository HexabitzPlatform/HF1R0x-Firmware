#include "BOS_MessageParser.h"
#include "BOS_MessageCodes.h"
#include <iostream>

BOSStatus BOS_MessageParser::parseMessage(const std::vector<uint8_t> &payload)
{
    if (payload.size() < 4)
    {
        std::cerr << "Invalid payload: too short\n";
        return BOSStatus::BOS_ERROR;
    }

    // Extract fields
    uint8_t destination = payload[0];
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
    case BOSMessageCode::CODE_UNKNOWN_MESSAGE:
        std::cerr << "Unknown message code: 0x" << std::hex << static_cast<int>(payload[3]) << "\n";
        break;

        // Indicator-Related Message Codes:
    case BOSMessageCode::CODE_PING:
        handlePingCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_CODE_IND_ON:
        handleIndicatorOnCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_IND_OFF:
        handleIndicatorOffCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_IND_TOGGLE:
        handleIndicatorToggleCode(destination, source, params);
        break;

        // Explore-Related Message Codes:
    case BOSMessageCode::CODE_HI:
        handleHiCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_HI_RESPONSE:
        handleHiResponseCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_EXPLORE_ADJ:
        handleExploreADJCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_EXPLORE_ADJ_RESPONSE:
        handleExploreADJResponseCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_PORT_DIRECTION:
        handlePortDirectionCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_MODULE_IDE:
        handleModuleIDCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_TOPOLOGY:
        handleTopologyCode(destination, source, params);
        break;

        // Read/Write Remote-Related Message Codes:
    case BOSMessageCode::CODE_READ_REMOTE:
        handleReadRemoteCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_READ_REMOTE_RESPONSE:
        handleReadRemoteResponseCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_WRITE_REMOTE:
        handleWriteRemoteCode(destination, source, params);
        break;

    case BOSMessageCode::CODE_WRITE_REMOTE_RESPONSE:
        handleWriteRemoteResponseCode(destination, source, params);
        break;

        // Power-Related Message Codes:
    case BOSMessageCode::ENABLE_STOP_MODE_UARTX:
        handleEnableStopModeCode(destination, source, params);
        break;

    default:
        // Delegate to subclass
        handleUnknownCode(destination, source, code, params);
        break;
    }

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/*************************************  BOS Message Codes Functions *******************************/
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handlePingCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0x01] Received from module " << static_cast<int>(source)
              << " with " << params.size() << " bytes of parameters\n";
    // Interpret params accordingly

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleIndicatorOnCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0x02] Execute special task from module " << static_cast<int>(source) << "\n";
    // Handle as needed

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleIndicatorOffCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleIndicatorToggleCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleHiCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleHiResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleExploreADJCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleExploreADJResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handlePortDirectionCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleModuleIDCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleTopologyCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleReadRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleReadRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleWriteRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleWriteRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleEnableStopModeCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/***********************************  Modules Message Codes Functions *****************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleUnknownCode(uint8_t dst, uint8_t src, BOSMessageCode code, const std::vector<uint8_t> &params)
{
    switch (code)
    {
    case BOSMessageCode::CODE_H01R0_ON:
        handleH01R0_ONCode(dst, src, params);
        break;
    case BOSMessageCode::CODE_H01R0_OFF:
        handleH01R0_OFFCode(dst, src, params);
        break;
    case BOSMessageCode::CODE_H01R0_TOGGLE:
        handleH01R0_ToggleCode(dst, src, params);
        break;
    case BOSMessageCode::CODE_H01R0_COLOR:
        handleH01R0_ColorCode(dst, src, params);
        break;
    case BOSMessageCode::CODE_H01R0_PULSE:
        handleH01R0_PulseCode(dst, src, params);
        break;
    case BOSMessageCode::CODE_H01R0_SWEEP:
        handleH01R0_SweepCode(dst, src, params);
        break;
    case BOSMessageCode::CODE_H01R0_DIM:
        handleH01R0_DIMCode(dst, src, params);
        break;

    default:
        std::cerr << "Module: Unhandled code.\n";
        return BOSStatus::BOS_ERROR;
    }
}

/**************************************************************************************************/
/* H01R0 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH01R0_ONCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH01R0_OFFCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH01R0_ToggleCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH01R0_ColorCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH01R0_PulseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH01R0_SweepCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH01R0_DIMCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Code 0xFF] Debug / special use\n";
    // Example logic

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/* H0BR4 Message Codes Functions ******************************************************************/
/**************************************************************************************************/

/**************************************************************************************************/
/* H08R7 Message Codes Functions ******************************************************************/
/**************************************************************************************************/