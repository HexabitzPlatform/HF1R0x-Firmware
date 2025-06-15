
#include "BOS.h"

int main()
{

    // Initialize GPIO PIN on Raspberry
    led.blink(LEDConfig::INITIAL_BLINK_TIMES, LEDConfig::BLINK_DELAY_MS);

    // Initialize UART on Raspberry Pi UART port (TX=GPIO14, RX=GPIO15, baudrate=115200)
    Porting::initUART(UARTConfig::TX_PIN, UARTConfig::RX_PIN, UARTConfig::BAUDRATE);

    std::cout << "UART initialized. Listening for BOS messages ...\n";

    Module_MessageParser bosParser;
    // BOS_MessageParser bosParser;
    UARTParser uartParser;

    // Connect UARTParser to BOS message parser
    uartParser.onMessageReceived([&bosParser](const std::vector<uint8_t> &payload)
                                 {
                                             std::cout << "\nValid BOS message received. Passing to BOS parser...\n";
                                             bosParser.parseMessage(payload); });

    // Setup UART receive callback to feed bytes into UARTParser
    Porting::setUartReceiveCallback([&uartParser](char byte)
                                    { uartParser.feed(static_cast<uint8_t>(byte)); });

    // Initialize BOS
    // initBOS();

    // led.blink(4 , 100);
    std::vector<uint8_t> Parameters = {2};

    Messaging::SendMessagetoModule(1, BOSMessageCode::CODE_H01R0_ON, Parameters);

    // Keep main thread alive indefinitely to allow background UART reading thread to run
    while (true)
    {
        // led.blink(4 , 100);
        // Messaging::SendMessagetoModule(1, BOSMessageCode::CODE_PING, {});
        Messaging::SendMessagetoModule(1, BOSMessageCode::CODE_H0BR4_SAMPLE_GYRO, Parameters);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        Messaging::SendMessagetoModule(1, BOSMessageCode::CODE_H0BR4_SAMPLE_ACC, Parameters);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        Messaging::SendMessagetoModule(1, BOSMessageCode::CODE_H0BR4_SAMPLE_MAG, Parameters);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        Messaging::SendMessagetoModule(1, BOSMessageCode::CODE_H0BR4_SAMPLE_TEMP, Parameters);

        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    return 0;
}
