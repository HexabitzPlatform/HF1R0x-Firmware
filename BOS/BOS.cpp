// #include <thread>
// #include <chrono>
// #include <iostream>
#include "BOS.h"

BOSOptionByte_t OptionByte;

// Initialize GPIO PIN on Raspberry
LED led(LEDConfig::GPIO_PIN);

/**************************************************************************************************/
/************************************  General Functions Definitions ******************************/
/**************************************************************************************************/
void initBOS(void)
{

    // Initialize GPIO PIN on Raspberry
    led.blink(LEDConfig::INITIAL_BLINK_TIMES, LEDConfig::BLINK_DELAY_MS);

    // Initialize UART on Raspberry Pi UART port (TX=GPIO14, RX=GPIO15, baudrate=115200)
    Porting::initUART(UARTConfig::TX_PIN, UARTConfig::RX_PIN, UARTConfig::BAUDRATE);

    std::cout << "UART initialized. Listening for BOS messages...\n";

    BOS_MessageParser bosParser;
    UARTParser uartParser;

    // Connect UARTParser to BOS message parser
    uartParser.onMessageReceived([&bosParser](const std::vector<uint8_t> &payload)
                                 {
                                         std::cout << "Valid BOS message received. Passing to BOS parser...\n";
                                         bosParser.parseMessage(payload); });

    // Setup UART receive callback to feed bytes into UARTParser
    Porting::setUartReceiveCallback([&uartParser](char byte)
                                    { uartParser.feed(static_cast<uint8_t>(byte)); });
};

/**************************************************************************************************/
void DisplayTopology(void)
{
    BOS_MessageParser object;

    std::cout << "There are " << static_cast<int>(object.NumberofModules) << " Modules including myself.\n";

    std::cout << "Module's Part Number      ID" << "\n";

    for (uint8_t row = 0; row < object.NumberofModules; row++)
    {
        std::cout << to_string(static_cast<ModulePN>(object.Array[row][0])) << "          " << row << "\n";
    }
}

/**************************************************************************************************/
std::string to_string(ModulePN pn)
{
    switch (pn)
    {
    case ModulePN::H01R0:
        return "H01R0";
    case ModulePN::P01R0:
        return "P01R0";
    case ModulePN::H23R0:
        return "H23R0";
    case ModulePN::H23R1:
        return "H23R1";
    case ModulePN::H23R3:
        return "H23R3";
    case ModulePN::H07R3:
        return "H07R3";
    case ModulePN::H08R6:
        return "H08R6";
    case ModulePN::P08R6:
        return "P08R6";
    case ModulePN::H09R0:
        return "H09R0";
    case ModulePN::H09R9:
        return "H09R9";
    case ModulePN::H1BR6:
        return "H1BR6";
    case ModulePN::H12R0:
        return "H12R0";
    case ModulePN::H13R7:
        return "H13R7";
    case ModulePN::H0FR1:
        return "H0FR1";
    case ModulePN::H0FR6:
        return "H0FR6";
    case ModulePN::H0FR7:
        return "H0FR7";
    case ModulePN::H1AR2:
        return "H1AR2";
    case ModulePN::H0AR9:
        return "H0AR9";
    case ModulePN::H1DR1:
        return "H1DR1";
    case ModulePN::H1DR5:
        return "H1DR5";
    case ModulePN::H0BR4:
        return "H0BR4";
    case ModulePN::H18R0:
        return "H18R0";
    case ModulePN::H26R0:
        return "H26R0";
    case ModulePN::H15R0:
        return "H15R0";
    case ModulePN::H10R4:
        return "H10R4";
    case ModulePN::H2AR3:
        return "H2AR3";
    case ModulePN::H41R6:
        return "H41R6";
    case ModulePN::H3BR6:
        return "H3BR6";
    case ModulePN::H18R1:
        return "H18R1";
    case ModulePN::H1FR5:
        return "H1FR5";
    case ModulePN::H3BR2:
        return "H3BR2";
    case ModulePN::H21R2:
        return "H21R2";
    case ModulePN::H17R1:
        return "H17R1";
    case ModulePN::H15R8:
        return "H15R8";
    case ModulePN::H2BR0:
        return "H2BR0";
    case ModulePN::H05R0:
        return "H05R0";
    case ModulePN::H3BR7:
        return "H3BR7";
    case ModulePN::H2BR1:
        return "H2BR1";
    case ModulePN::H07R8:
        return "H07R8";
    case ModulePN::H08R7:
        return "H08R7";
    case ModulePN::H16R6:
        return "H16R6";
    case ModulePN::P08R7:
        return "P08R7";
    case ModulePN::H19R0:
        return "H19R0";
    case ModulePN::Raspberry_PI:
        return "RPI  ";
    case ModulePN::H14RA:
        return "H14RA";
    default:
        return "Unknown";
    }
}

/**************************************************************************************************/
