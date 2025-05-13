#include "BOS.h"
#include "BOS_MessageParser.h" // Will be used when dispatch logic is added
#include "Porting.h"           // For hardware-related initialization
#include "UARTParser.h"        // For UART setup and message handling

#include <iostream> // For debug output
#include <thread>   // For possible background tasks
#include <chrono>   // For delays

namespace BOS
{
    // Forward declaration of message handler
    void handleBOSMessage(const std::vector<uint8_t> &message);

    void initBOS()
    {
        std::cout << "[BOS] Initializing system..." << std::endl;

        // Create a static UARTParser instance
        static UARTParser parser;

        // Register callback for valid messages
        parser.onMessageReceived([](const std::vector<uint8_t> &message)
                                 {
            std::cout << "[BOS] Received message of size " << message.size() << "\n";
            handleBOSMessage(message); });

        // Start UART receiving thread
        std::thread uartThread([&parser]()
                               {
            while (true)
            {
                int byte = uartReceiveByte(); // from Porting.cpp
                if (byte >= 0)
                {
                    parser.feed(static_cast<uint8_t>(byte));
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } });

        uartThread.detach(); // Let the thread run independently
    }

    // Handles a full validated BOS message
    void handleBOSMessage(const std::vector<uint8_t> &message)
    {
        // Pass message to BOS message parser
        processBOSMessage(message);
    }
}