// Porting/Porting.h
#ifndef PORTING_H
#define PORTING_H

#include <string>
#include <functional>

namespace Porting
{
    void initGPIO(int pin);
    void writeGPIO(int pin, bool value);
    bool readGPIO(int pin);

    // Initializes UART with given TX/RX GPIO pins and baudrate
    void initUART(int txPin, int rxPin, int baudrate);

    // Sends a string over UART
    void uartSend(const std::string &message);

    // Starts receiving UART data with a callback per character
    void uartReceive(const std::function<void(char)> &onReceiveChar);

    void setUartReceiveCallback(std::function<void(char)> callback); // <-- Add this line

}

#endif
