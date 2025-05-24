#pragma once

#include <vector>
#include <cstdint>
// #include "BOS.h"

// class BOS_MessageParser
// {
// public:
//     virtual BOSStatus parseMessage(const std::vector<uint8_t> &payload);

// private:
//     // Indicator-Related Message Codes Functions:
//     BOSStatus handlePingCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleIndicatorOnCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleIndicatorOffCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleIndicatorToggleCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

//     // Explore-Related Message Codes Functions:
//     BOSStatus handleHiCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleHiResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleExploreADJCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleExploreADJResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handlePortDirectionCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleModuleIDCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleTopologyCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

//     // Read/Write Remote-Related Message Codes Functions:
//     BOSStatus handleReadRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleReadRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleWriteRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
//     BOSStatus handleWriteRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

//     // Power-Related Message Codes Functions:
//     BOSStatus handleEnableStopModeCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

//     // Overridable method to handle unknown/module-specific codes
//     virtual BOSStatus handleModuleMessageCode(uint8_t dst, uint8_t src, BOSMessageCode code, const std::vector<uint8_t> &params);
// };

// class Module_MessageParser : public BOS_MessageParser
// {
// protected:
//     BOSStatus handleModuleMessageCode(uint8_t dst, uint8_t src, BOSMessageCode code, const std::vector<uint8_t> &params) override;

// private:
//     // H05R0x - 1S Lipo Charger w/ USB-C
//     BOSStatus handleH05R0_CellVoltageCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH05R0_CellCurrentCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH05R0_CellPowerCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH05R0_CellTemperatureCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH05R0_CellCapacityCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH05R0_CellAgeCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH05R0_CellCyclesCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);

//     // H08R7 - TOF
//     BOSStatus handleH08R7_SampleCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);

//     // H0BR4 - IMU
//     BOSStatus handleH0BR4_GyroCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH0BR4_AccCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH0BR4_MagCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
//     BOSStatus handleH0BR4_TemperatureCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
// };
