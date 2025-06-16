// #include "vector"
// #include "BOS_Messaging.h"
#include "BOS.h"

namespace Messaging
{
    BOSStatus SendMessagetoModule(uint8_t dstID, BOSMessageCode code, const std::vector<uint8_t> &params)
    {
        std::vector<uint8_t> packet;

        // Start bytes
        packet.push_back('H'); // 0x48
        packet.push_back('Z'); // 0x5A

        // Convert enum to raw uint16_t value
        uint16_t rawCode = static_cast<uint16_t>(code);

        // Determine if we need 1 or 2 bytes for the code
        bool isExtendedCode = rawCode > 0xFF;
        uint8_t codeLen = isExtendedCode ? 2 : 1;

        // Options byte (only Extended Code flag for now)
        uint8_t options = 0x00;
        if (isExtendedCode)
            options |= 0x02;

        // Calculate BOS-compliant Length (excluding H, Z, Length, and CRC)
        uint8_t length = 3 + codeLen + params.size(); // dest + src + options + code + params
        packet.push_back(length);

        // Add message fields
        packet.push_back(dstID);
        // uint8_t src = 2;
        packet.push_back(PIConfig::piID);
        packet.push_back(options);

        // Add message code
        packet.push_back(static_cast<uint8_t>(rawCode & 0xFF)); // LSB
        if (isExtendedCode)
            packet.push_back(static_cast<uint8_t>((rawCode >> 8) & 0xFF)); // MSB

        // Add parameters
        if (!params.empty())
            packet.insert(packet.end(), params.begin(), params.end());

        // Compute and add CRC8
        uint8_t crc = calculateCRC8(packet);
        packet.push_back(crc);

        Porting::uartSend(packet);

        

        return BOSStatus::BOS_OK;
    }

    BOSStatus SendDataRequestToModule(uint8_t dstID, BOSMessageCode code)
    {
        // std::vector<uint8_t> Parameters = {2};
        std::vector<uint8_t> Parameters = {PIConfig::piID};

        SendMessagetoModule(dstID, code, Parameters);

        return BOSStatus::BOS_OK;
    }

};
