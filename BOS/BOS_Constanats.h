#pragma once

#include <array>
#include <cstdint>
#include <cstring>

namespace PIConfig
{
    constexpr uint8_t piPort = 1;
    constexpr uint8_t piPartNumber = 44; /*ModulePartNumbers_e in BOS.h in Modules */

    inline uint8_t piID = 1;
};

namespace LEDConfig
{
    constexpr uint8_t GPIO_PIN = 23;       // GPIO pin connected to LED
    constexpr int BLINK_DELAY_MS = 100;    // Delay in milliseconds for blinking
    constexpr int INITIAL_BLINK_TIMES = 2; // Number of times to blink
}

namespace UARTConfig
{
    constexpr uint8_t TX_PIN = 14;        // GPIO pin for UART TX
    constexpr uint8_t RX_PIN = 15;        // GPIO pin for UART RX
    constexpr uint32_t BAUDRATE = 921600; // UART baud rate
}

class BOSMessageCodec
{
public:
    // Convert float to 4 bytes
    static std::array<uint8_t, 4> floatToBytes(float value)
    {
        std::array<uint8_t, 4> bytes{};
        std::memcpy(bytes.data(), &value, sizeof(float));
        return bytes;
    }

    // Convert 4 bytes to float
    static float bytesToFloat(const std::array<uint8_t, 4> &bytes)
    {
        float value;
        std::memcpy(&value, bytes.data(), sizeof(float));
        return value;
    }

    // Convert 4 bytes to int
    static int bytesToInt(const std::array<uint8_t, 4> &bytes)
    {
        int value;
        std::memcpy(&value, bytes.data(), sizeof(int));
        return value;
    }

    /*
    // Encode
    std::array<uint8_t, 4> encoded = BOSMessageCodec::floatToBytes(original);

    // Decode
    float decoded = BOSMessageCodec::bytesToFloat(encoded);
    */
};
