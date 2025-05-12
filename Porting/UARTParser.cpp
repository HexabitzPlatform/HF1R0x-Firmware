// Porting/UARTParser.cpp
#include "UARTParser.h"
#include <iostream> // For std::cerr, std::endl
#include <iomanip>  // For std::hex, std::dec

void UARTParser::onMessageReceived(Callback cb)
{
    callback = cb;
}

void UARTParser::feed(uint8_t byte)
{
    switch (state)
    {
    case State::WaitForH:
        if (byte == 'H')
        {
            buffer.clear();
            buffer.push_back(byte);
            state = State::WaitForZ;
        }
        break;

    case State::WaitForZ:
        if (byte == 'Z')
        {
            buffer.push_back(byte);
            state = State::WaitForLength;
        }
        else
        {
            state = State::WaitForH;
        }
        break;

    case State::WaitForLength:
        buffer.push_back(byte);
        expectedLength = byte;
        state = State::Collecting;
        break;

    case State::Collecting:
        buffer.push_back(byte);
        if (buffer.size() == expectedLength + 4)
        { // +4 for H, Z, Length, CRC

            std::vector<uint8_t> crcData(buffer.begin() + 2, buffer.end() - 1); // Length to last param
            uint8_t receivedCRC = buffer.back();
            uint8_t calculatedCRC = calculateCRC8(crcData);

            if (calculatedCRC == receivedCRC)
            {
                if (callback)
                    callback(buffer);
            }
            else
            {
                std::cerr << "CRC mismatch: expected 0x" << std::hex << (int)calculatedCRC
                          << ", got 0x" << (int)receivedCRC << std::dec << std::endl;
            }

            state = State::WaitForH;
        }
        break;
    }
}

uint8_t calculateCRC8(const std::vector<uint8_t> &data)
{
    uint8_t crc = 0x00;
    for (uint8_t byte : data)
    {
        crc ^= byte;
        for (int i = 0; i < 8; ++i)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}
