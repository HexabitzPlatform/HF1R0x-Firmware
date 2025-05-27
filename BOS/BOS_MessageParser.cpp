#include <iostream>
#include "BOS.h"

// Define a 2D array: 6 rows, 2 columns
// std::array<std::array<uint16_t, 2>, 6> NeighborsInfo{};
std::array<uint16_t, 2> NeighborsInfo{};
// std::array<uint8_t, 46> MessageParames{};
std::vector<uint8_t> MessageParames;

std::array<uint16_t, 2> Array{};

std::vector<uint8_t> RxBuffer;

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
BOSStatus BOS_MessageParser::handleModuleMessageCode(uint8_t dst, uint8_t src, BOSMessageCode code, const std::vector<uint8_t> &params)
{
    std::cerr << "Base BOS_MessageParser: Unhandled module-specific code\n";
    return BOSStatus::BOS_ERROR;
}

/**************************************************************************************************/
/***********************************  Modules Message Codes Functions *****************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleModuleMessageCode(uint8_t dst, uint8_t source, BOSMessageCode code, const std::vector<uint8_t> &params)
{
    switch (code)
    {
        /* H05R0 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H05R0_CELLVOLTAGE:
        handleH05R0_CellVoltageCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLCURRENT:
        handleH05R0_CellCurrentCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLPOWER:
        handleH05R0_CellPowerCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLTEMPERATURE:
        handleH05R0_CellTemperatureCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLCAPACITY:
        handleH05R0_CellCapacityCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLAGE:
        handleH05R0_CellAgeCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLCYCLES:
        handleH05R0_CellCyclesCode(dst, source, params);
        break;

        /* H08R7 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H08R7_SAMPLE_PORT:
        handleH08R7_SampleCode(dst, source, params);
        break;
        /* H09R0 Message Codes  *******************************************************************/

        /* H09R9 Message Codes  *******************************************************************/

        /* H0AR9 Message Codes  *******************************************************************/

        /* H0BR4 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H0BR4_SAMPLE_GYRO:
        handleH0BR4_GyroCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H0BR4_SAMPLE_ACC:
        handleH0BR4_AccCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H0BR4_SAMPLE_MAG:
        handleH0BR4_MagCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H0BR4_SAMPLE_TEMP:
        handleH0BR4_TemperatureCode(dst, source, params);
        break;

        /* H21R2 Message Codes  *******************************************************************/

        /* H2AR3 Message Codes  *******************************************************************/

        /* For testing only */
    case BOSMessageCode::CODE_READ_RESPONSE:
        handleH0BR4_GyroCode(dst, source, params);
        break;

    default:
        std::cerr << "Module: Unhandled code.\n";
        return BOSStatus::BOS_ERROR;
    }
    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/* H05R0 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellVoltageCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCurrentCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellPowerCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellTemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCapacityCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellAgeCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCyclesCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/* H08R7 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH08R7_SampleCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/* H0BR4 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_GyroCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float GyroX = 0.0f;
    float GyroY = 0.0f;
    float GyroZ = 0.0f;

    // Expected layout:
    // params[0] = BOS status
    // params[1] = format
    // params[2] = count
    // params[3..6] = GyroX
    // params[7..10] = GyroY
    // params[11..14] = GyroZ

    if (params.size() < 15)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[3], params[4], params[5], params[6]};
    std::array<uint8_t, 4> yBytes = {params[7], params[8], params[9], params[10]};
    std::array<uint8_t, 4> zBytes = {params[11], params[12], params[13], params[14]};

    GyroX = BOSMessageCodec::bytesToFloat(xBytes);
    GyroY = BOSMessageCodec::bytesToFloat(yBytes);
    GyroZ = BOSMessageCodec::bytesToFloat(zBytes);

    // std::cout << "Module PN: " << static_cast<int>(ModulePN::H0BR4) << "\n";
    std::cout << "[Sample Gyroscope] Received from Module: " << to_string(ModulePN::H0BR4)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << " GyroX: " << GyroX << "\n GyroY: " << GyroY << "\n GyroZ: " << GyroZ << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_AccCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float AccX = 0.0f;
    float AccY = 0.0f;
    float AccZ = 0.0f;

    if (params.size() < 15)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[3], params[4], params[5], params[6]};
    std::array<uint8_t, 4> yBytes = {params[7], params[8], params[9], params[10]};
    std::array<uint8_t, 4> zBytes = {params[11], params[12], params[13], params[14]};

    AccX = BOSMessageCodec::bytesToFloat(xBytes);
    AccY = BOSMessageCodec::bytesToFloat(yBytes);
    AccZ = BOSMessageCodec::bytesToFloat(zBytes);

    // std::cout << "Module PN: " << static_cast<int>(ModulePN::H0BR4) << "\n";
    std::cout << "[Sample Accelerometer] Received from Module: " << to_string(ModulePN::H0BR4)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << " AccX: " << AccX << "\n AccY: " << AccY << "\n AccZ: " << AccZ << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_MagCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float MagX = 0.0f;
    float MagY = 0.0f;
    float MagZ = 0.0f;

    if (params.size() < 15)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[3], params[4], params[5], params[6]};
    std::array<uint8_t, 4> yBytes = {params[7], params[8], params[9], params[10]};
    std::array<uint8_t, 4> zBytes = {params[11], params[12], params[13], params[14]};

    MagX = BOSMessageCodec::bytesToFloat(xBytes);
    MagY = BOSMessageCodec::bytesToFloat(yBytes);
    MagZ = BOSMessageCodec::bytesToFloat(zBytes);

    // std::cout << "Module PN: " << static_cast<int>(ModulePN::H0BR4) << "\n";
    std::cout << "[Sample Magnetometer] Received from Module: " << to_string(ModulePN::H0BR4)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << " MagX: " << MagX << "\n MagY: " << MagY << "\n MagZ: " << MagZ << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float Temp = 0.0f;

    if (params.size() < 7)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[3], params[4], params[5], params[6]};

    Temp = BOSMessageCodec::bytesToFloat(xBytes);

    // std::cout << "Module PN: " << static_cast<int>(ModulePN::H0BR4) << "\n";
    std::cout << "[Sample Magnetometer] Received from Module: " << to_string(ModulePN::H0BR4)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << " Temp: " << Temp << "Celsius\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/