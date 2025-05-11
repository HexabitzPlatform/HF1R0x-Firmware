// src/main.cpp
#include "Porting.h"
#include <iostream>
#include <unistd.h> // for sleep
#include <cstring>  // for strlen

int main()
{
    const int ledPin = 17; // GPIO17 (BCM numbering)
    const std::string message = "Hello from Raspberry Pi!\n";

    std::cout << "Initializing peripherals...\n";

    // Send a message over UART
    Porting::uartSend(message);

    // Register a receive callback
    Porting::setUartReceiveCallback([](char c)
                                    { std::cout << "Received char over UART: " << c << std::endl; });

    std::cout << "Blinking LED on GPIO" << ledPin << " and sending UART data...\n";

    // Blink LED forever and send periodic messages over UART
    while (true)
    {
        Porting::writeGPIO(ledPin, true); // LED ON
        Porting::uartSend("LED ON\n");
        sleep(1);

        Porting::writeGPIO(ledPin, false); // LED OFF
        Porting::uartSend("LED OFF\n");
        sleep(1);
    }

    return 0;
}
