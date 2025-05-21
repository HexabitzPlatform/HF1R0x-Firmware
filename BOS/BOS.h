#pragma once

/* C++ Libraries */
#include <vector>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>

/* BOS Files */
#include "Porting.h"
#include "UARTParser.h"
#include "BOS_Messaging.h"
#include "BOS_Constanats.h"
#include "BOS_MessageParser.h"
#include "BOS_MessageCodes.h"

/* */
enum class BOSStatus : uint8_t
{
    // BOS Status:
    BOS_OK = 0,                 /* Operation successful */
    BOS_ERR_UnknownMessage = 1, /* Unknown message received */
    BOS_ERR_NoResponse = 2,     /* No response from module */
    BOS_ERR_MSG_Reflection = 3, /* Message reflection detected */
    BOS_ERR_UnIDedModule = 5,   /* Unidentified module */

    BOS_ERR_Keyword = 6,       /* Invalid keyword */
    BOS_ERR_ExistingAlias = 7, /* Alias already exists */

    BOS_ERR_REMOTE_READ_TIMEOUT = 15,  /* Timeout during remote read */
    BOS_ERR_REMOTE_READ_NO_VAR = 16,   /* No variable found during remote read */
    BOS_ERR_REMOTE_WRITE_TIMEOUT = 17, /* Timeout during remote write */
    BOS_ERR_REMOTE_WRITE_INDEX = 19,   /* Invalid remote write index */
    BOS_ERR_LOCAL_FORMAT_UPDATED = 20, /* Local format updated */
    BOS_ERR_REMOTE_WRITE_ADDRESS = 21, /* Invalid remote write address */

    BOS_ERR_PORT_BUSY = 23,         /* Communication port busy */
    BOS_ERR_TIMEOUT = 24,           /* Operation timeout */
    BOS_ERR_WrongName = 100,        /* Incorrect name */
    BOS_ERR_WrongGroup = 101,       /* Incorrect group */
    BOS_ERR_WrongID = 102,          /* Incorrect ID */
    BOS_ERR_WrongParam = 103,       /* Incorrect parameter */
    BOS_ERR_WrongValue = 104,       /* Incorrect value */
    BOS_ERR_MSG_DOES_NOT_FIT = 105, /* Message does not fit */

    BOS_MULTICAST = 254, /* Multicast message */
    BOS_BROADCAST = 255, /* Broadcast message */

    BOS_ERROR = 255 /* Generic error */

    // Module Status:
};

/* Define module PN strings */
enum class ModulePN : char
{
    H01R0,
    P01R0,
    H23R0,
    H23R1,
    H23R3,
    H07R3,
    H08R6,
    P08R6,
    H09R0,
    H09R9,
    H1BR6,
    H12R0,
    H13R7,
    H0FR1,
    H0FR6,
    H0FR7,
    H1AR2,
    H0AR9,
    H1DR1,
    H1DR5,
    H0BR4,
    H18R0,
    H26R0,
    H15R0,
    H10R4,
    H2AR3,
    H41R6,
    H3BR6,
    H18R1,
    H1FR5,
    H3BR2,
    H21R2,
    H17R1,
    H15R8,
    H2BR0,
    H05R0,
    H3BR7,
    H2BR1,
    H07R8,
    H08R7,
    H16R6,
    P08R7,
    H19R0
};

class LED
{
private:
    uint8_t gpioPin;
    bool state = false;

public:
    explicit LED(uint8_t _pin) : gpioPin(_pin)
    {
        Porting::initGPIO(gpioPin);
    }

    void on()
    {
        Porting::writeGPIO(gpioPin, true);
        state = true;
    }

    void off()
    {
        Porting::writeGPIO(gpioPin, false);
        state = false;
    }

    void toggle()
    {
        state = !state;
        Porting::writeGPIO(gpioPin, state);
    }

    void blink(int times, int delay_ms)
    {
        for (int i = 0; i < times; ++i)
        {
            on();
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
            off();
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }
};

void initBOS();