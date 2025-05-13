#ifndef PORTING_H
#define PORTING_H

#include <string>
#include <functional>

namespace Porting
{
    // ==============================
    // GPIO Functions
    // ==============================

    /// @brief Initialize GPIO pin (as output or input, implementation-defined)
    void initGPIO(int pin);

    /// @brief Write a digital value to GPIO pin
    void writeGPIO(int pin, bool value);

    /// @brief Read a digital value from GPIO pin
    bool readGPIO(int pin);

    // ==============================
    // UART Functions
    // ==============================

    /// @brief Initializes UART with given TX/RX GPIO pins and baudrate
    void initUART(int txPin, int rxPin, int baudrate);

    /// @brief Sends a string over UART
    void uartSend(const std::string &message);

    /// @brief Starts receiving UART data with a callback per character
    void uartReceive(const std::function<void(char)> &onReceiveChar);

    /// @brief Sets the UART receive callback explicitly (advanced)
    void setUartReceiveCallback(std::function<void(char)> callback);

    // ==============================
    // UART Byte Receive Function
    // ==============================

    /// @brief Receives one byte from UART (blocking or non-blocking, implementation-defined)
    int uartReceiveByte(); // Add this declaration for uartReceiveByte()
}

#endif // PORTING_H
