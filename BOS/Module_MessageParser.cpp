#include <vector>
#include <cstdint>
#include <iostream>
#include "BOS.h"
// #include "BOS_MessageParser.h"

// std::vector<uint8_t> RxBuffer;

// /**************************************************************************************************/
// /***********************************  Modules Message Codes Functions *****************************/
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleModuleMessageCode(uint8_t dst, uint8_t source, BOSMessageCode code, const std::vector<uint8_t> &params)
// {
//     switch (code)
//     {
//         /* H05R0 Message Codes  *******************************************************************/
//     case BOSMessageCode::CODE_H05R0_CELLVOLTAGE:
//         handleH05R0_CellVoltageCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H05R0_CELLCURRENT:
//         handleH05R0_CellCurrentCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H05R0_CELLPOWER:
//         handleH05R0_CellPowerCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H05R0_CELLTEMPERATURE:
//         handleH05R0_CellTemperatureCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H05R0_CELLCAPACITY:
//         handleH05R0_CellCapacityCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H05R0_CELLAGE:
//         handleH05R0_CellAgeCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H05R0_CELLCYCLES:
//         handleH05R0_CellCyclesCode(dst, source, params);
//         break;

//         /* H08R7 Message Codes  *******************************************************************/
//     case BOSMessageCode::CODE_H08R7_SAMPLE_PORT:
//         handleH08R7_SampleCode(dst, source, params);
//         break;
//         /* H09R0 Message Codes  *******************************************************************/

//         /* H09R9 Message Codes  *******************************************************************/

//         /* H0AR9 Message Codes  *******************************************************************/

//         /* H0BR4 Message Codes  *******************************************************************/
//     case BOSMessageCode::CODE_H0BR4_SAMPLE_GYRO:
//         handleH0BR4_GyroCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H0BR4_SAMPLE_ACC:
//         handleH0BR4_AccCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H0BR4_SAMPLE_MAG:
//         handleH0BR4_MagCode(dst, source, params);
//         break;

//     case BOSMessageCode::CODE_H0BR4_SAMPLE_TEMP:
//         handleH0BR4_TemperatureCode(dst, source, params);
//         break;

//         /* H21R2 Message Codes  *******************************************************************/

//         /* H2AR3 Message Codes  *******************************************************************/

//     default:
//         std::cerr << "Module: Unhandled code.\n";
//         return BOSStatus::BOS_ERROR;
//     }
//     return BOSStatus::BOS_OK;
// }

// /**************************************************************************************************/
// /* H05R0 Message Codes Functions ******************************************************************/
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH05R0_CellVoltageCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH05R0_CellCurrentCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH05R0_CellPowerCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH05R0_CellTemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH05R0_CellCapacityCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH05R0_CellAgeCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH05R0_CellCyclesCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }

// /**************************************************************************************************/
// /* H08R7 Message Codes Functions ******************************************************************/
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH08R7_SampleCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }

// /**************************************************************************************************/
// /* H0BR4 Message Codes Functions ******************************************************************/
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH0BR4_GyroCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {
//     float GyroX = 0.0f;
//     float GyroY = 0.0f;
//     float GyroZ = 0.0f;

//     // Expected layout:
//     // params[0] = BOS status
//     // params[1] = format
//     // params[2] = count
//     // params[3..6] = GyroX
//     // params[7..10] = GyroY
//     // params[11..14] = GyroZ

//     if (params.size() < 15)
//         return BOSStatus::BOS_ERROR;

//     std::array<uint8_t, 4> xBytes = {params[3], params[4], params[5], params[6]};
//     std::array<uint8_t, 4> yBytes = {params[7], params[8], params[9], params[10]};
//     std::array<uint8_t, 4> zBytes = {params[11], params[12], params[13], params[14]};

//     GyroX = BOSMessageCodec::bytesToFloat(xBytes);
//     GyroY = BOSMessageCodec::bytesToFloat(yBytes);
//     GyroZ = BOSMessageCodec::bytesToFloat(zBytes);

//     std::cout << "[Sample Gyro Code] Received from Module: " << static_cast<int>(source) << "\n";
//     std::cout << "GyroX: " << GyroX << ", GyroY: " << GyroY << ", GyroZ: " << GyroZ << "\n";

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH0BR4_AccCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH0BR4_MagCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/
// BOSStatus Module_MessageParser::handleH0BR4_TemperatureCode(uint8_t dst, uint8_t source, const std::vector<uint8_t> &params)
// {

//     return BOSStatus::BOS_OK;
// }
// /**************************************************************************************************/