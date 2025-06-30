#include <vector>
#include <cstdint>
#include <iostream>
#include "BOS.h"

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
    case BOSMessageCode::CODE_H09R9_SAMPLE_TEMP:
        handleH09R9_TemperatureCode(dst, source, params);
        break;

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

        /* H1FR5 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H1FR5_GET_POSITION:
        handleH1FR5_PositionCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H1FR5_GET_UTC:
        handleH1FR5_UTCCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H1FR5_GET_SPEED:
        handleH1FR5_SpeedCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H1FR5_GET_HEIGHT:
        handleH1FR5_HeightCode(dst, source, params);
        break;

        /* H2AR3 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H2AR3_SAMPLE_VOLT:
        handleH2AR3_VoltCode(dst, source, params);
        break;

    case BOSMessageCode::CODE_H2AR3_SAMPLE_CURRENT:
        handleH2AR3_CurrentCode(dst, source, params);
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
    CellVoltageResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    result.voltage = BOSMessageCodec::bytesToFloat(bytes);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCurrentCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    CellCurrentResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    result.current = BOSMessageCodec::bytesToFloat(bytes);

    result.batteryState = (result.current >= 0) ? Batterystate::charging : Batterystate::discharging;

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellPowerCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    CellPowerResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    result.power = BOSMessageCodec::bytesToFloat(bytes);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellTemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    CellTempResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    result.temp = BOSMessageCodec::bytesToFloat(bytes);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCapacityCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    CellCapacityResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    result.capacity = BOSMessageCodec::bytesToFloat(bytes);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_StateofChargeCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    SOCResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    result.SOC = BOSMessageCodec::bytesToFloat(bytes);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellAgeCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    CellAgeResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    result.age = BOSMessageCodec::bytesToFloat(bytes);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCyclesCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    CellCyclesResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    result.cycles = BOSMessageCodec::bytesToFloat(bytes);

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
/* H09R9 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH09R9_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float temp = 0.0f;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> bytes = {params[5], params[6], params[7], params[8]};

    temp = BOSMessageCodec::bytesToFloat(bytes);

    std::cout << "[Sample Temperature] Received from Module: " << to_string(ModulePN::H09R9)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Temperature: " << temp << " Celsius\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
/* H0AR9 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_ColorCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    ColorResult result;

    if (params.size() < 11)
        return (result.status = BOSStatus::BOS_ERROR);

    std::array<uint8_t, 2> redBytes = {params[5], params[6]};
    std::array<uint8_t, 2> greenBytes = {params[7], params[8]};
    std::array<uint8_t, 2> blueBytes = {params[9], params[10]};

    result.red = BOSMessageCodec::bytesToUint16_t(redBytes);
    result.green = BOSMessageCodec::bytesToUint16_t(greenBytes);
    result.blue = BOSMessageCodec::bytesToUint16_t(blueBytes);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0AR9::colorPromise.set_value(result);

    return result.status;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_DistanceCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    DistanceResult result;

    if (params.size() < 7)
        return (result.status = BOSStatus::BOS_ERROR);

    std::array<uint8_t, 2> distanceBytes = {params[5], params[6]};

    result.distance = BOSMessageCodec::bytesToUint16_t(distanceBytes);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0AR9::DistancePromise.set_value(result);

    return result.status;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    TempResult result;

    if (params.size() < 9)
        return (result.status = BOSStatus::BOS_ERROR);

    std::array<uint8_t, 4> tempBytes = {params[5], params[6], params[7], params[8]};

    result.temp = BOSMessageCodec::bytesToFloat(tempBytes);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0AR9::TempPromise.set_value(result);

    return result.status;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_HumidityCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    HumidityResult result;

    if (params.size() < 9)
        return (result.status = BOSStatus::BOS_ERROR);

    std::array<uint8_t, 4> humidityBytes = {params[5], params[6], params[7], params[8]};

    result.humidity = BOSMessageCodec::bytesToFloat(humidityBytes);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0AR9::HumidityPromise.set_value(result);

    return result.status;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0AR9_PIRCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    PIRResult result;

    if (params.size() < 6)
        return (result.status = BOSStatus::BOS_ERROR);

    result.pir = params.at(5);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0AR9::PIRPromise.set_value(result);

    return result.status;
}
/**************************************************************************************************/
/* H0BR4 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_GyroCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    GyroResult result;

    if (params.size() < 17)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[5], params[6], params[7], params[8]};
    std::array<uint8_t, 4> yBytes = {params[9], params[10], params[11], params[12]};
    std::array<uint8_t, 4> zBytes = {params[13], params[14], params[15], params[16]};

    result.x = BOSMessageCodec::bytesToFloat(xBytes);
    result.y = BOSMessageCodec::bytesToFloat(yBytes);
    result.z = BOSMessageCodec::bytesToFloat(zBytes);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0BR4::GyroPromise.set_value(result);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_AccCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    AccResult result;

    if (params.size() < 17)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[5], params[6], params[7], params[8]};
    std::array<uint8_t, 4> yBytes = {params[9], params[10], params[11], params[12]};
    std::array<uint8_t, 4> zBytes = {params[13], params[14], params[15], params[16]};

    result.x = BOSMessageCodec::bytesToFloat(xBytes);
    result.y = BOSMessageCodec::bytesToFloat(yBytes);
    result.z = BOSMessageCodec::bytesToFloat(zBytes);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0BR4::AccPromise.set_value(result);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_MagCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    MagResult result;

    if (params.size() < 17)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[5], params[6], params[7], params[8]};
    std::array<uint8_t, 4> yBytes = {params[9], params[10], params[11], params[12]};
    std::array<uint8_t, 4> zBytes = {params[13], params[14], params[15], params[16]};

    result.x = BOSMessageCodec::bytesToInt(xBytes);
    result.y = BOSMessageCodec::bytesToInt(yBytes);
    result.z = BOSMessageCodec::bytesToInt(zBytes);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0BR4::MagPromise.set_value(result);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    IMU_TempResult result;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> xBytes = {params[5], params[6], params[7], params[8]};

    result.temp = BOSMessageCodec::bytesToFloat(xBytes);
    result.status = BOSStatus::BOS_OK;

    // Set the promise result to unblock the waiting thread
    H0BR4::TempPromise.set_value(result);

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
/* H1FR5 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH1FR5_PositionCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH1FR5_UTCCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH1FR5_SpeedCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH1FR5_HeightCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
/* H2AR3 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH2AR3_VoltCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float volt = 0;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> voltBytes = {params[5], params[6], params[7], params[8]};

    volt = BOSMessageCodec::bytesToFloat(voltBytes);

    std::cout << "[Sample RMS Volt] Received from Module: " << to_string(ModulePN::H2AR3)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "Volt: " << volt << " Volt\n\n";

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH2AR3_CurrentCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
{
    float current = 0;

    if (params.size() < 9)
        return BOSStatus::BOS_ERROR;

    std::array<uint8_t, 4> currentBytes = {params[5], params[6], params[7], params[8]};

    current = BOSMessageCodec::bytesToFloat(currentBytes);

    std::cout << "[Sample RMS current] Received from Module: " << to_string(ModulePN::H2AR3)
              << " , ID: " << static_cast<int>(source) << "\n";
    std::cout << "current: " << current << " Amp\n\n";

    return BOSStatus::BOS_OK;
}