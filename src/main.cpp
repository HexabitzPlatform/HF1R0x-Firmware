#include "Porting.h"
#include "UARTParser.h"
#include <thread>
#include <chrono>
#include <iostream>

int main()
{
    const int ledPin = 17; // GPIO17 (BCM numbering)

    std::cout << "Blinking LED and receiving UART data...\n";

    Porting::initUART(14, 15, 115200); // Adjust TX/RX pins as needed

    static UARTParser parser;

    // Define what to do when a full valid message is received
    parser.onMessageReceived([](const std::vector<uint8_t>& message) {
        std::cout << "Received valid BOS packet: ";
        for (uint8_t byte : message) {
            printf("%02X ", byte);
        }
        std::cout << std::endl;
    });

    Porting::uartReceive([=](char c) {
        parser.feed(static_cast<uint8_t>(c));
    });

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        Porting::writeGPIO(ledPin, true); // LED ON
        std::this_thread::sleep_for(std::chrono::seconds(1));
        Porting::writeGPIO(ledPin, false); // LED OFF
    }

    return 0;
}
