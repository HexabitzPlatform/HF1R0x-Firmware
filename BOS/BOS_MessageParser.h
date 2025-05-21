#ifndef BOS_MESSAGE_PARSER_H
#define BOS_MESSAGE_PARSER_H

#include <vector>
#include <cstdint>
#include "BOS.h"

class BOS_MessageParser
{
public:
    virtual BOSStatus parseMessage(const std::vector<uint8_t> &payload);

private:
    // Indicator-Related Message Codes Functions:
    BOSStatus handlePingCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorOnCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorOffCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorToggleCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Explore-Related Message Codes Functions:
    BOSStatus handleHiCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleHiResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleExploreADJCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleExploreADJResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handlePortDirectionCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleModuleIDCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleTopologyCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Read/Write Remote-Related Message Codes Functions:
    BOSStatus handleReadRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleReadRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleWriteRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleWriteRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Power-Related Message Codes Functions:
    BOSStatus handleEnableStopModeCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Overridable method to handle unknown/module-specific codes
    virtual BOSStatus handleModuleMessageCode(uint8_t dst, uint8_t src, BOSMessageCode code, const std::vector<uint8_t> &params);
};

class Module_MessageParser : public BOS_MessageParser
{
protected:
    BOSStatus handleModuleMessageCode(uint8_t dst, uint8_t src, BOSMessageCode code, const std::vector<uint8_t> &params) override;

private:
    // H01R0x - RGB
    BOSStatus handleH01R0_ONCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
    BOSStatus handleH01R0_OFFCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
    BOSStatus handleH01R0_ToggleCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
    BOSStatus handleH01R0_ColorCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
    BOSStatus handleH01R0_PulseCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
    BOSStatus handleH01R0_SweepCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);
    BOSStatus handleH01R0_DIMCode(uint8_t dst, uint8_t src, const std::vector<uint8_t> &params);

    // H0BR4 - IMU

    // H08R7 - TOF
};

#endif // BOS_MESSAGE_PARSER_H
