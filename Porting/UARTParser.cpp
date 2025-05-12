#include "UARTParser.h"
#include <iostream> // For std::cerr, std::endl
#include <iomanip>  // For std::hex, std::dec

// Sets the callback function to be called when a valid message is received
void UARTParser::onMessageReceived(Callback cb)
{
    callback = cb; // Store the provided callback
}

// Processes a single incoming byte, updating the parser's state
void UARTParser::feed(uint8_t byte)
{
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
            // Extract data for CRC (from Length to byte before CRC)
            std::vector<uint8_t> crcData(buffer.begin() + 2, buffer.end() - 1);
            uint8_t receivedCRC = buffer.back();            // Last byte is the received CRC
            uint8_t calculatedCRC = calculateCRC8(crcData); // Calculate CRC

            // Validate CRC
            if (calculatedCRC == receivedCRC)
            {
                if (callback)
                {
                    // Only pass the payload (excluding Length and CRC)
                    std::vector<uint8_t> payload(crcData.begin() + 1, crcData.end());
                    callback(payload); // Call callback with valid payload
                }
            }
            else
            {
                // Log CRC mismatch error with hex values
                std::cerr << "CRC mismatch: expected 0x" << std::hex << (int)calculatedCRC
                          << ", got 0x" << (int)receivedCRC << std::dec << std::endl;
            }

            state = State::WaitForH; // Reset to start for next message
        }
        break;
    }
}

// Calculates an 8-bit CRC for the input data using polynomial 0x07
uint8_t calculateCRC8(const std::vector<uint8_t> &data)
{
    uint8_t crc = 0x00; // Initialize CRC to 0
    // Process each byte in the data
    for (uint8_t byte : data)
    {
        crc ^= byte; // XOR byte with current CRC
        // Process each bit
        for (int i = 0; i < 8; ++i)
        {
            if (crc & 0x80)              // If MSB is 1
                crc = (crc << 1) ^ 0x07; // Shift left and XOR with polynomial
            else
                crc <<= 1; // Shift left
        }
    }
    return crc; // Return final CRC
}
