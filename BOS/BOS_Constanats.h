#pragma once

#include <cstdint>

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
