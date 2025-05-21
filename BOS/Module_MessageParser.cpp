#include <vector>
#include <cstdint>
#include <iostream>
#include "BOS.h"
#include "BOS_MessageParser.h"

/**************************************************************************************************/
/***********************************  Modules Message Codes Functions *****************************/
/**************************************************************************************************/
BOSStatus Module_MessageParser::handleModuleMessageCode(uint8_t dst, uint8_t src, BOSMessageCode code, const std::vector<uint8_t> &params)
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
    return BOSStatus::BOS_OK;
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
