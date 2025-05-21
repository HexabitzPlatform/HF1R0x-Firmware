
#include "BOS.h"

int main()
{
    // Initialize the GPIO pin
    int ledPin = 23; // GPIO23
    Porting::initGPIO(ledPin);

    // // Initialize UART on Raspberry Pi UART port (TX=GPIO14, RX=GPIO15, baudrate=115200)
    // Porting::initUART(14, 15, 921600);

    // std::cout << "? UART initialized. Listening for BOS messages from hardware...\n";

    // BOS_MessageParser bosParser;
    // UARTParser uartParser;

    // // Connect UARTParser to BOS message parser
    // uartParser.onMessageReceived([&bosParser](const std::vector<uint8_t> &payload)
    //                              {
    //                                  std::cout << "?? Valid BOS message received. Passing to BOS parser...\n";
    //                                  bosParser.parseMessage(payload); });

    // // Setup UART receive callback to feed bytes into UARTParser
    // Porting::setUartReceiveCallback([&uartParser](char byte)
    //                                 { uartParser.feed(static_cast<uint8_t>(byte)); });

    // Send test packet once (Loopback Test)
    // std::vector<uint8_t> test_packet = {0x48, 0x5A, 0x04, 0x02, 0x01, 0x00, 0x01, 0xE3};
    // Porting::uartSend(std::string(test_packet.begin(), test_packet.end()));

    initBOS();

    // Keep main thread alive indefinitely to allow background UART reading thread to run
    while (true)
    {
        // Messaging::SendMessagetoModule(2, BOSMessageCode::CODE_PING, {});

        // Turn LED ON
        std::cout << "Turning LED ON\n";
        Porting::writeGPIO(ledPin, true);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Turn LED OFF
        std::cout << "Turning LED OFF\n";
        Porting::writeGPIO(ledPin, false);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}

// #include <iostream>
// #include "buildBOSPacket.cpp"

// int main() {
//     std::vector<uint8_t> params = {0x01, 0x02}; // Example parameters
//     uint16_t code = 0x1234;
//     uint8_t destination = 0x01;

//     auto packet = buildBOSPacket(destination, code, params);

//     std::cout << "BOS Packet to send:\n";
//     for (uint8_t b : packet) {
//         std::cout << "0x" << std::hex << static_cast<int>(b) << " ";
//     }
//     std::cout << std::endl;

//     // Now send 'packet' over your UART device
// }