#include "UARTParser.h"
#include <iostream>
#include <iomanip>
#include <cmath>

// Sets the callback function to be called when a valid message is received
void UARTParser::onMessageReceived(Callback cb)
{
    callback = cb; // Store the provided callback
}

// Processes a single incoming byte, updating the parser's state
void UARTParser::feed(uint8_t byte)
{
    std::cout << "Feeding byte: " << std::hex << static_cast<int>(byte) << std::endl;

    // State machine to handle byte-by-byte parsing
    switch (state)
    {
    // Waiting for the start byte 'H'
    case State::WaitForH:
        if (byte == 'H')
        {
            buffer.clear();          // Clear buffer to start a new message
            buffer.push_back(byte);  // Store 'H'
            state = State::WaitForZ; // Move to next state
        }
        break;

    // Waiting for the second byte 'Z'
    case State::WaitForZ:
        if (byte == 'Z')
        {
            buffer.push_back(byte);       // Store 'Z'
            state = State::WaitForLength; // Move to next state
        }
        else
        {
            state = State::WaitForH; // Invalid byte, reset to start
        }
        break;

    // Waiting for the length byte
    case State::WaitForLength:
        buffer.push_back(byte);    // Store the length byte
        expectedLength = byte;     // Set expected payload length
        state = State::Collecting; // Move to collecting payload
        break;

    // Collecting payload and CRC bytes
    case State::Collecting:
        buffer.push_back(byte); // Add byte to buffer
        // Check if we've received the full message (H, Z, Length, Payload, CRC)
        if (buffer.size() == expectedLength + 4)
        {
            // Extract data for CRC (from Delemeter Bytes to byte before CRC)
            std::vector<uint8_t> crcData(buffer.begin(), buffer.end() - 1);

            // Pad to 4-byte blocks for CRC function
            while (crcData.size() % 4 != 0)
                crcData.push_back(0);

            uint8_t receivedCRC = buffer.back();             // Last byte is the received CRC
            uint8_t calculatedCRC = calculateCRC32(crcData); // Calculate CRC

            // Compare
            if (calculatedCRC == receivedCRC)
            {
                // Success: extract payload (length to before CRC)
                std::vector<uint8_t> payload(buffer.begin() + 3, buffer.end() - 1);
                if (callback)
                    callback(payload);
            }
            else
            {
                std::cerr << "CRC mismatch: expected 0x" << std::hex << (int)calculatedCRC
                          << ", got 0x" << (int)receivedCRC << std::dec << std::endl;
            }

            state = State::WaitForH; // Reset to start for next message
        }
        break;
    }
}

// Calculates an 8-bit CRC for the input data using polynomial 0x07
uint8_t calculateCRC32(const std::vector<uint8_t> &data)
{
    unsigned int crc = 0xFFFFFFFF;
    const size_t length = data.size();

    // Calculate ceil(length / 4)
    double l = std::ceil(double(length) / 4.0);
    if (l <= 2)
        l = 2;

    // Prepare reversed data in 4-byte chunks
    std::vector<uint8_t> reversed(length, 0);
    size_t shift = 0;
    for (size_t i = 0; i < (size_t)l; ++i)
    {
        for (int jj = 3; jj >= 0; --jj)
        {
            size_t src_index = shift + jj;
            size_t dst_index = shift + (3 - jj);
            if (src_index < length)
                reversed[dst_index] = data[src_index];
            else
                reversed[dst_index] = 0;
        }
        shift += 4;
    }

    // Compute CRC over reversed data
    for (size_t i = 0; i < (size_t)(l * 4); ++i)
    {
        crc ^= ((unsigned int)reversed[i]) << 24;
        for (int j = 0; j < 8; ++j)
        {
            unsigned int msb = crc >> 31;
            crc <<= 1;
            crc ^= (0 - msb) & 0x4C11DB7;
        }
    }

    // Return only the lowest byte (CRC8)
    return static_cast<uint8_t>(crc & 0xFF);
}