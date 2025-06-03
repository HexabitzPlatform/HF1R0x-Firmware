#include <iostream>
#include "BOS.h"

// Define a 2D array: 6 rows, 2 columns
// std::array<std::array<uint16_t, 2>, 6> NeighborsInfo{};
// std::array<uint16_t, 2> NeighborsInfo{};
// // std::array<uint8_t, 46> MessageParames{};
// std::vector<uint8_t> MessageParames;
// std::array<uint16_t, 2> Array{};

BOSStatus BOS_MessageParser::parseMessage(const std::vector<uint8_t> &payload)
{
    // Extract fields
    uint8_t destination = payload[0];
    uint8_t source = payload[1];
    uint8_t option = payload[2];
    BOSMessageCode code;
    std::vector<uint8_t> params;

    if (payload.size() < 4)
    {
        std::cerr << "Invalid payload: too short\n";
        return BOSStatus::BOS_ERROR;
    }

    /* Assign the value of option byte to OptionByte structure */
    *(uint8_t *)&OptionByte = option;

    /* Check option byte */
    if (OptionByte.ExtendedMessageCode)
    {
        code = static_cast<BOSMessageCode>(
            (static_cast<uint16_t>(payload.at(4) << 8)) | (static_cast<uint16_t>(payload.at(3))));

        params.insert(params.end(), payload.begin() + 5, payload.end());
    }
    else
    {
        code = static_cast<BOSMessageCode>(payload[3]);

        params.insert(params.end(), payload.begin() + 4, payload.end());
    }

    // BOSMessageCode code = static_cast<BOSMessageCode>(payload[3]);

    // if (payload.size() > 4)
    // {
    //     params.insert(params.end(), payload.begin() + 4, payload.end());
    // }

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

    default:
        // Delegate to subclass
        handleModuleMessageCode(destination, source, code, params);
        break;
    }

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/*************************************  BOS Message Codes Functions *******************************/
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handlePingCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    // Interpret params accordingly
    led.blink(LEDConfig::INITIAL_BLINK_TIMES, LEDConfig::BLINK_DELAY_MS);

    std::cout << "[Ping Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleIndicatorOnCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    led.on();
    std::cout << "[LED on Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleIndicatorOffCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    led.off();
    std::cout << "[LED off Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleIndicatorToggleCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    led.toggle();
    std::cout << "[LED toggle Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleHiCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    /* Record neighbor info */
    NeighborsInfo[0] = (static_cast<uint16_t>(source) << 8) | static_cast<uint16_t>(params[2]);    /* Neighbor ID + Neighbor own port */
    NeighborsInfo[1] = (static_cast<uint16_t>(params[0]) << 8) | static_cast<uint16_t>(params[1]); /* Neighbor PN */

    std::cout << "[Hi Code] Reveived from Module: " << static_cast<int>(source)
              << "which PN: " << NeighborsInfo[1] << "and its ID: " << params[0];

    /* Send Raspberry PI info */
    // MessageParames[0] = (static_cast<uint8_t>(PIConfig::piPartNumber) << 8);
    // MessageParames[1] = (static_cast<uint8_t>(PIConfig::piPartNumber));
    // MessageParames[2] = PIConfig::piPort;

    MessageParames.push_back((PIConfig::piPartNumber)); /* LSB of PN */
    MessageParames.push_back(0);                        /* MSB of PN: to match Neighbor array in BOS which expects 16-bit */
    MessageParames.push_back(PIConfig::piPort);

    Messaging::SendMessagetoModule(0, BOSMessageCode::CODE_HI_RESPONSE, MessageParames);

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleHiResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Hi Response Code] Reveived from Module: " << static_cast<int>(source) << "\n";
    /* Record your neighbor info */
    /* this message code is important for only a master module that runs Explore function
       so, raspberry will never be a master explore */

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleExploreADJCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Explore ADJ Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    /* Send back a message indicates that no modules are connected to the raspberry other the the master */

    Messaging::SendMessagetoModule(source, BOSMessageCode::CODE_EXPLORE_ADJ_RESPONSE, MessageParames);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleExploreADJResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Explore ADJ Response Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handlePortDirectionCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Port Direction Code] Reveived from Module: " << static_cast<int>(source)
              << "Raspberry can not Implement this\n";

    /* Raspberry Pi can not swap UART Pins. so, instead we'll try fix this in master BOS Module */

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleModuleIDCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    PIConfig::piID = (params.at(0));

    std::cout << "[ Module ID Code] Reveived from Module: " << static_cast<int>(source)
              << "Update Raspberry Pi ID to be: " << PIConfig::piID << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleTopologyCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    static uint8_t longMessageScratchpad[2];
    uint16_t longMessageLastPtr = 0;

    if (OptionByte.LongMessage)
    {
        /* Array is 2-byte oriented thus memcpy can copy only even number of bytes
         * TODO test maybe broken */
        /* Use a 1-byte oriented scratchpad */
        memcpy(&longMessageScratchpad[0] + longMessageLastPtr, params.data(), params.size());
        longMessageLastPtr += params.size();
    }
    else
    {
        memcpy(&longMessageScratchpad[0] + longMessageLastPtr, params.data(), params.size());
        longMessageLastPtr += params.size();
        // N = (longMessageLastPtr / (1 + 1)) / 2;

        /* Copy the scratchpad to Array */
        memcpy(&Array, &longMessageScratchpad, longMessageLastPtr);
        longMessageLastPtr = 0;

        led.blink(2, 100);
    }

    std::cout << "[Topology Code] Reveived from Module: " << static_cast<int>(source)
              << "Update Topology to be:" << params.at(0) << "\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleReadRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Read Remote Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleReadRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Read Remote Response Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleWriteRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Write Remote Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
BOSStatus BOS_MessageParser::handleWriteRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params)
{
    std::cout << "[Write Remote Response Code] Reveived from Module: " << static_cast<int>(source) << "\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
// default implementation
// the linker still needs the base class's definition � unless it's marked as = 0 (pure virtual).
BOSStatus BOS_MessageParser::handleModuleMessageCode(uint8_t dst, uint8_t source, BOSMessageCode code, const std::vector<uint8_t> &params)
{
    std::cerr << "Base BOS_MessageParser: Unhandled module-specific code\n";
    return BOSStatus::BOS_ERROR;
}
