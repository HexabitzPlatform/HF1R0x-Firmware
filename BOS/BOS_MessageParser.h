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
    BOSStatus handleCode01(uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleCode02(uint8_t source, const std::vector<uint8_t> &params);
    BOSStatus handleCodeFF(uint8_t source, const std::vector<uint8_t> &params);
};

#endif // BOS_MESSAGE_PARSER_H
