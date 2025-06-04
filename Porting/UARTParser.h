#pragma once

#include <functional>
#include <vector>
#include <cstdint>

// UARTParser class: Parses incoming UART data to extract valid messages
class UARTParser
{
public:
    // Define a callback type: a function that takes a vector of bytes (the message) and returns void
    using Callback = std::function<void(const std::vector<uint8_t> &)>;

    // Feeds a single byte to the parser for processing
    void feed(uint8_t byte);

    // Registers a callback function to be called when a complete message is received
    void onMessageReceived(Callback cb);

private:
    // Enum to track the parser's state during message processing
    enum class State
    {
        WaitForH,              // Waiting for the 'H' start byte
        WaitForZ,              // Waiting for the 'Z' second byte
        WaitForLength,         // Waiting for the length byte
        Collecting             // Collecting payload and CRC bytes
    } state = State::WaitForH; // Initial state is waiting for 'H'

    // Buffer to store the incoming message bytes
    std::vector<uint8_t> buffer;

    // Expected length of the payload, set by the length byte
    size_t expectedLength = 0;

    // Stores the user-provided callback function
    Callback callback;
};

// Function to calculate an 8-bit CRC for message validation
uint8_t calculateCRC32(const std::vector<uint8_t> &data);