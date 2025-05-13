#ifndef BOS_MESSAGE_PARSER_H
#define BOS_MESSAGE_PARSER_H

#include <vector>
#include <cstdint>

class BOS_MessageParser
{
public:
    void parseMessage(const std::vector<uint8_t> &payload);

private:
    void handleCode01(uint8_t source, const std::vector<uint8_t> &params);
    void handleCode02(uint8_t source, const std::vector<uint8_t> &params);
    void handleCodeFF(uint8_t source, const std::vector<uint8_t> &params);
};

#endif // BOS_MESSAGE_PARSER_H
