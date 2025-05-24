#include <vector>
#include <cstdint>
#include <iostream>
#include "BOS.h"
// #include "BOS_MessageParser.h"

/**************************************************************************************************/
/***********************************  Modules Message Codes Functions *****************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleModuleMessageCode(uint8_t dst, uint8_t src, BOSMessageCode code, const std::vector<uint8_t> &params)
{
    switch (code)
    {
        /* H05R0 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H05R0_CELLVOLTAGE:
        handleH05R0_CellVoltageCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLCURRENT:
        handleH05R0_CellCurrentCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLPOWER:
        handleH05R0_CellPowerCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLTEMPERATURE:
        handleH05R0_CellTemperatureCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLCAPACITY:
        handleH05R0_CellCapacityCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLAGE:
        handleH05R0_CellAgeCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H05R0_CELLCYCLES:
        handleH05R0_CellCyclesCode(dst, src, params);
        break;

        /* H08R7 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H08R7_SAMPLE_PORT:
        handleH08R7_SampleCode(dst, src, params);
        break;
        /* H09R0 Message Codes  *******************************************************************/

        /* H09R9 Message Codes  *******************************************************************/

        /* H0AR9 Message Codes  *******************************************************************/

        /* H0BR4 Message Codes  *******************************************************************/
    case BOSMessageCode::CODE_H0BR4_SAMPLE_GYRO:
        handleH0BR4_GyroCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H0BR4_SAMPLE_ACC:
        handleH0BR4_AccCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H0BR4_SAMPLE_MAG:
        handleH0BR4_MagCode(dst, src, params);
        break;

    case BOSMessageCode::CODE_H0BR4_SAMPLE_TEMP:
        handleH0BR4_TemperatureCode(dst, src, params);
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
BOSStatus Module_MessageParser::handleH05R0_CellVoltageCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCurrentCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellPowerCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellTemperatureCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCapacityCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellAgeCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH05R0_CellCyclesCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/* H08R7 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH08R7_SampleCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}

/**************************************************************************************************/
/* H0BR4 Message Codes Functions ******************************************************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_GyroCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_AccCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_MagCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleH0BR4_TemperatureCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params)
{

    return BOSStatus::BOS_OK;
}
/**************************************************************************************************/