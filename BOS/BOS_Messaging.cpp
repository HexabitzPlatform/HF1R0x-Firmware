#include "vector"
#include "UARTParser.h"
#include "BOS_Messaging.h"

// Create a valid BOS message packet
std::vector<uint8_t> buildBOSPacket(uint8_t dst, uint16_t code, const std::vector<uint8_t> &params, uint8_t src = 1)
{
    std::vector<uint8_t> packet;

    // Start bytes
    packet.push_back('H'); // 0x48
    packet.push_back('Z'); // 0x5A

    // Determine if we need 1 or 2 bytes for the code
    bool isExtendedCode = code > 0xFF;
    uint8_t codeLen = isExtendedCode ? 2 : 1;

    // Options byte (only Extended Code flag for now)
    uint8_t options = 0x00;
    if (isExtendedCode)
        options |= 0x02;

    // Calculate BOS-compliant Length (excluding H, Z, Length, and CRC)
    uint8_t length = 3 + codeLen + params.size(); // dest + src + options + code + params
    packet.push_back(length);

    // Add message fields
    packet.push_back(dst);
    packet.push_back(src);
    packet.push_back(options);

    // Add message code
    packet.push_back(static_cast<uint8_t>(code & 0xFF)); // LSB
    if (isExtendedCode)
        packet.push_back(static_cast<uint8_t>((code >> 8) & 0xFF)); // MSB

    // Add parameters
    packet.insert(packet.end(), params.begin(), params.end());

    // Compute and add CRC8
    uint8_t crc = computeCRC8(packet);
    packet.push_back(crc);

    return packet;
}