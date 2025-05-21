#include "BOS.h"
#include "UARTParser.h"
#include "Porting.h"           // Needed for uartReceiveByte()
#include "BOS_MessageParser.h" // Needed for processBOSMessage()

#include <thread>
#include <chrono>
#include <iostream>

void initBOS()
{

    // Initialize GPIO as an output on Raspberry
    LED LED(LEDConfig::GPIO_PIN);
    LED.blink(LEDConfig::INITIAL_BLINK_TIMES, LEDConfig::BLINK_DELAY_MS);

    // Initialize UART on Raspberry Pi UART port (TX=GPIO14, RX=GPIO15, baudrate=115200)
    Porting::initUART(UARTConfig::TX_PIN, UARTConfig::RX_PIN, UARTConfig::BAUDRATE);

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
