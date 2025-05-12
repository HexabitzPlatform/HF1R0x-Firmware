// #include "Porting.h"
// #include "UARTParser.h"
// #include <thread>
// #include <chrono>
// #include <iostream>

// int main()
// {
//     const int ledPin = 17; // GPIO17 (BCM numbering)

//     std::cout << "Blinking LED and receiving UART data...\n";

//     Porting::initUART(14, 15, 115200); // Adjust TX/RX pins as needed

//     static UARTParser parser;

//     // Define what to do when a full valid message is received
//     parser.onMessageReceived([](const std::vector<uint8_t>& message) {
//         std::cout << "Received valid BOS packet: ";
//         for (uint8_t byte : message) {
//             printf("%02X ", byte);
//         }
//         std::cout << std::endl;
//     });

//     Porting::uartReceive([=](char c) {
//         parser.feed(static_cast<uint8_t>(c));
//     });

//     while (true)
//     {
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//         Porting::writeGPIO(ledPin, true); // LED ON
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//         Porting::writeGPIO(ledPin, false); // LED OFF
//     }

//     return 0;
// }

// Main program to demonstrate UARTParser without hardware UART
#include "UARTParser.h"
#include <iostream> // For std::cout, std::hex, std::endl
#include <iomanip>  // For std::setw, std::setfill

// Callback function to handle received messages
void handleMessage(const std::vector<uint8_t> &message)
{
    std::cout << "Received valid message: ";
    // Print each byte in hex format with leading zeros
    for (uint8_t byte : message)
    {
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
    }
    std::cout << std::dec << std::endl; // Reset to decimal for future outputs
}

int main()
{
    // Create an instance of UARTParser
    UARTParser parser;

    // Register the callback function
    parser.onMessageReceived(handleMessage);

    // Simulate a valid UART message: ['H', 'Z', Length=0x03, 0x01, 0x02, 0x03, CRC]
    std::cout << "Feeding valid message..." << std::endl;
    uint8_t validMessage[] = {'H', 'Z', 0x03, 0x01, 0x02, 0x03, 0x00};
    // Calculate CRC for the payload (Length, 0x01, 0x02, 0x03)
    std::vector<uint8_t> crcData = {0x03, 0x01, 0x02, 0x03};
    validMessage[6] = calculateCRC8(crcData); // Set correct CRC
    // Feed each byte to the parser
    for (uint8_t byte : validMessage)
    {
        parser.feed(byte);
    }

    // Simulate an invalid UART message (same payload, wrong CRC)
    std::cout << "\nFeeding invalid message (wrong CRC)..." << std::endl;
    uint8_t invalidMessage[] = {'H', 'Z', 0x03, 0x01, 0x02, 0x03, 0xFF}; // Incorrect CRC
    for (uint8_t byte : invalidMessage)
    {
        parser.feed(byte);
    }

    return 0;
}