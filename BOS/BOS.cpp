#include "BOS.h"
#include "UARTParser.h"
#include "Porting.h"           // Needed for uartReceiveByte()
#include "BOS_MessageParser.h" // Needed for processBOSMessage()

#include <thread>
#include <chrono>
#include <iostream>

// namespace BOS
// {
//     void initBOS()
//     {
//         static UARTParser parser;

//         parser.onMessageReceived([](const std::vector<uint8_t> &message)
//                                  { handleBOSMessage(message); });

//         std::thread uartThread([]()
//                                {
//             while (true)
//             {
//                 int byte = Porting::uartReceiveByte(); // Call with Porting:: namespace
//                 if (byte >= 0)
//                 {
//                     static UARTParser parser;
//                     parser.feed(static_cast<uint8_t>(byte));
//                 }
//                 std::this_thread::sleep_for(std::chrono::milliseconds(1));
//             } });

//         uartThread.detach(); // Let it run in the background
//     }

//     void handleBOSMessage(const std::vector<uint8_t> &message)
//     {
//         std::cout << "[BOS] Received valid message of size " << message.size() << "\n";
//         processBOSMessage(message);
//     }

//     void processBOSMessage(const std::vector<unsigned char> &message)
//     {
//         // Your implementation of message processing
//     }
// }

void initBOS()
{

    // Initialize UART on Raspberry Pi UART port (TX=GPIO14, RX=GPIO15, baudrate=115200)
    Porting::initUART(14, 15, 921600);

    std::cout << "? UART initialized. Listening for BOS messages from hardware...\n";

    BOS_MessageParser bosParser;
    UARTParser uartParser;

    // Connect UARTParser to BOS message parser
    uartParser.onMessageReceived([&bosParser](const std::vector<uint8_t> &payload)
                                 {
                                         std::cout << "?? Valid BOS message received. Passing to BOS parser...\n";
                                         bosParser.parseMessage(payload); });

    // Setup UART receive callback to feed bytes into UARTParser
    Porting::setUartReceiveCallback([&uartParser](char byte)
                                    { uartParser.feed(static_cast<uint8_t>(byte)); });
};
