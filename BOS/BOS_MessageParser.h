#ifndef BOS_MESSAGE_PARSER_H
#define BOS_MESSAGE_PARSER_H

#include <vector>
#include <cstdint>
#include "BOS.h"

class BOS_MessageParser
{
public:
    BOSStatus parseMessage(const std::vector<uint8_t> &payload);

private:
    // Indicator-Related Message Codes:
    BOSStatus handlePingCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorOnCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorOffCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleIndicatorToggleCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Explore-Related Message Codes:
    BOSStatus handleHiCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleHiResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleExploreADJCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleExploreADJResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handlePortDirectionCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleModuleIDCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleTopologyCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Read/Write Remote-Related Message Codes:
    BOSStatus handleReadRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleReadRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleWriteRemoteCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleWriteRemoteResponseCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);

    // Power-Related Message Codes:
    BOSStatus handleEnableStopModeCode(uint8_t dts, uint8_t source, const std::vector<uint8_t> &params);
};

#endif // BOS_MESSAGE_PARSER_H
