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
BOSStatus BOS_MessageParser::handleModuleMessageCode(uint8_t dst, uint8_t source, BOSMessageCode code, const std::vector<uint8_t> &params)
{
    std::cerr << "Base BOS_MessageParser: Unhandled module-specific code\n";
    return BOSStatus::BOS_ERROR;
}

/**************************************************************************************************/
/***********************************  Modules Message Codes Functions *****************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleModuleMessageCode(uint8_t dst, uint8_t source, BOSMessageCode code, const std::vector<uint8_t> &params)
{

    // code: here is useless , because BOS modules responses to message code as one message: CODE_READ_RESPONSE = 46

    // Expected layout from params:
    // params[0] = BOS status
    // params[1] = format
    // params[2] = count
    // params[3] = message code LSB
    // params[4] = message code MSB

    BOSMessageCode originalCode = static_cast<BOSMessageCode>(params.at(3) | (params.at(4) << 8));

    switch (originalCode)
    {
        /* H05R0 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H05R0_CELL_VOLTAGE:
        handleH05R0_CellVoltageCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELL_CURRENT:
        handleH05R0_CellCurrentCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELL_POWER:
        handleH05R0_CellPowerCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELL_TEMPERATURE:
        handleH05R0_CellTemperatureCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELL_CAPACITY:
        handleH05R0_CellCapacityCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_STATE_OF_CHARGE:
        handleH05R0_StateofChargeCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELL_AGE:
        handleH05R0_CellAgeCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELL_CYCLES:
        handleH05R0_CellCyclesCode(dst, source, params);
        break;

        /* H08R7 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H08R7_SAMPLE_DISTANCE:
        handleH08R7_DistanceCode(dst, source, params);
        break;
        /* H09R0 Message Codes  *******************************************************************/

        /* H09R9 Message Codes  *******************************************************************/

        /* H0AR9 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H0AR9_SAMPLE_COLOR:
        handleH0AR9_ColorCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H0AR9_SAMPLE_DISTANCE:
        handleH0AR9_DistanceCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H0AR9_SAMPLE_TEMP:
        handleH0AR9_TemperatureCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H0AR9_SAMPLE_HUMIDITY:
        handleH0AR9_HumidityCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H0AR9_SAMPLE_PIR:
        handleH0AR9_PIRCode(dst, source, params);
        break;

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
    float voltage = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    voltage = BOSMessageCodec::bytesToFloat(bytes);

    std::cout << "[Sample Battery Voltage] Received from Module: " << to_string(ModulePN::H05R0)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Battery Voltage: " << voltage << "Volt\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCurrentCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    enum class state : uint8_t
    {
        charging = 0,
        discharging = 1
    };

    float current = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    current = BOSMessageCodec::bytesToFloat(bytes);

    state batteryState = (current >= 0) ? state::charging : state::discharging;

    std::cout << "[Sample Battery current] Received from Module: " << to_string(ModulePN::H05R0)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Battery Current: " << current << "Amp\n";
    std::cout << "Battery State: "
              << (batteryState == state::charging ? "Battery is charging" : "Battery is discharging") << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellPowerCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float power = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    power = BOSMessageCodec::bytesToFloat(bytes);

    std::cout << "[Sample Battery Power] Received from Module: " << to_string(ModulePN::H05R0)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Battery Power: " << power << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellTemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float temp = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    temp = BOSMessageCodec::bytesToFloat(bytes);

    std::cout << "[Sample Battery Temperature] Received from Module: " << to_string(ModulePN::H05R0)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Battery Temperature: " << temp << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCapacityCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float capacity = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    capacity = BOSMessageCodec::bytesToFloat(bytes);

    std::cout << "[Sample Battery Capacity] Received from Module: " << to_string(ModulePN::H05R0)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Battery Capacity: " << capacity << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_StateofChargeCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float SOC = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    SOC = BOSMessageCodec::bytesToFloat(bytes);

    std::cout << "[Sample Battery State of Charge] Received from Module: " << to_string(ModulePN::H05R0)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Battery State of Charge: " << SOC << "% \n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellAgeCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float age = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    age = BOSMessageCodec::bytesToFloat(bytes);

    std::cout << "[Sample Battery Age] Received from Module: " << to_string(ModulePN::H05R0)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Battery Age: " << age << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCyclesCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float cycles = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    cycles = BOSMessageCodec::bytesToFloat(bytes);

    std::cout << "[Sample Battery Cycles] Received from Module: " << to_string(ModulePN::H05R0)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Battery Cycles: " << cycles << "\n\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/* H08R7 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH08R7_DistanceCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    uint16_t distance = 0;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    distance = params.at(5) | (params.at(6) << 8);

    std::cout << "[Sample Distance] Received from Module: " << to_string(ModulePN::H08R7)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Distance: " << distance << "\n\n";

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/* H0AR9 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_ColorCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    uint16_t red = 0, green = 0, blue = 0;

    if (params.size() < 11)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 2> redBytes = {params[5], params[6]};
    std::array<uint8_t, 2> greenBytes = {params[7], params[8]};
    std::array<uint8_t, 2> blueBytes = {params[9], params[10]};

    red = BOSMessageCodec::bytesToUint16_t(redBytes);
    green = BOSMessageCodec::bytesToUint16_t(greenBytes);
    blue = BOSMessageCodec::bytesToUint16_t(blueBytes);

    std::cout << "[Sample Color] Received from Module: " << to_string(ModulePN::H0AR9)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Red: " << red << " \n";
    std::cout << "Green: " << green << " \n";
    std::cout << "Blue: " << blue << " \n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_DistanceCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    uint16_t distance = 0;

    if (params.size() < 7)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 2> distanceBytes = {params[5], params[6]};

    distance = BOSMessageCodec::bytesToUint16_t(distanceBytes);

    std::cout << "[Sample Distance] Received from Module: " << to_string(ModulePN::H0AR9)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Distance: " << distance << " \n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float temp = 0;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> tempBytes = {params[5], params[6], params[7], params[8]};

    temp = BOSMessageCodec::bytesToFloat(tempBytes);

    std::cout << "[Sample Temperature] Received from Module: " << to_string(ModulePN::H0AR9)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Temperature: " << temp << " Celsius\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_HumidityCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float humidity = 0;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> humidityBytes = {params[5], params[6], params[7], params[8]};

    humidity = BOSMessageCodec::bytesToFloat(humidityBytes);

    std::cout << "[Sample Humidity] Received from Module: " << to_string(ModulePN::H0AR9)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Humidity: " << humidity << " \n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_PIRCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    uint8_t PIR = 0;

    if (params.size() < 6)
        return BOSStatus::BOS_ERROR;

    PIR = params.at(5);

    std::cout << "[Sample PIR] Received from Module: " << to_string(ModulePN::H08R7)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "PIR: " << PIR << "\n\n";

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

    if (params.size() < 17)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[5], params[6], params[7], params[8]};
    std::array<uint8_t, 4> yBytes = {params[9], params[10], params[11], params[12]};
    std::array<uint8_t, 4> zBytes = {params[13], params[14], params[15], params[16]};

    GyroX = BOSMessageCodec::bytesToFloat(xBytes);
    GyroY = BOSMessageCodec::bytesToFloat(yBytes);
    GyroZ = BOSMessageCodec::bytesToFloat(zBytes);

    std::cout << "[Sample Gyroscope] Received from Module: " << to_string(ModulePN::H0BR4)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "GyroX: " << GyroX << "\nGyroY: " << GyroY << "\nGyroZ: " << GyroZ << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_AccCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float AccX = 0.0f;
    float AccY = 0.0f;
    float AccZ = 0.0f;

    if (params.size() < 17)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[5], params[6], params[7], params[8]};
    std::array<uint8_t, 4> yBytes = {params[9], params[10], params[11], params[12]};
    std::array<uint8_t, 4> zBytes = {params[13], params[14], params[15], params[16]};

    AccX = BOSMessageCodec::bytesToFloat(xBytes);
    AccY = BOSMessageCodec::bytesToFloat(yBytes);
    AccZ = BOSMessageCodec::bytesToFloat(zBytes);

    std::cout << "[Sample Accelerometer] Received from Module: " << to_string(ModulePN::H0BR4)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "AccX: " << AccX << "\nAccY: " << AccY << "\nAccZ: " << AccZ << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_MagCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    int MagX = 0.0f;
    int MagY = 0.0f;
    int MagZ = 0.0f;

    if (params.size() < 17)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[5], params[6], params[7], params[8]};
    std::array<uint8_t, 4> yBytes = {params[9], params[10], params[11], params[12]};
    std::array<uint8_t, 4> zBytes = {params[13], params[14], params[15], params[16]};

    MagX = BOSMessageCodec::bytesToInt(xBytes);
    MagY = BOSMessageCodec::bytesToInt(yBytes);
    MagZ = BOSMessageCodec::bytesToInt(zBytes);

    std::cout << "[Sample Magnetometer] Received from Module: " << to_string(ModulePN::H0BR4)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "MagX: " << MagX << "\nMagY: " << MagY << "\nMagZ: " << MagZ << "\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float Temp = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[5], params[6], params[7], params[8]};

    Temp = BOSMessageCodec::bytesToFloat(xBytes);

    std::cout << "[Sample Temperature] Received from Module: " << to_string(ModulePN::H0BR4)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Temp: " << Temp << " Celsius\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/