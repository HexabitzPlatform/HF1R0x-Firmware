#pragma once

#include <array>
#include <cstdint>
#include <cstring>

/* Raspberry Pi identity and port settings ********************************************************/
namespace PIConfig
{
    constexpr uint8_t piPort = 1;
    constexpr uint8_t piPartNumber = 44; /*ModulePartNumbers_e in BOS.h in Modules */

    inline uint8_t piID = 0;
};

/* LED-related constants **************************************************************************/
namespace LEDConfig
{
    constexpr uint8_t GPIO_PIN = 23;       // GPIO pin connected to LED
    constexpr int BLINK_DELAY_MS = 80;    // Delay in milliseconds for blinking
    constexpr int INITIAL_BLINK_TIMES = 1; // Number of times to blink
}

/* UART peripheral setup **************************************************************************/
namespace UARTConfig
{
    constexpr uint8_t TX_PIN = 14;        // GPIO pin for UART TX
    constexpr uint8_t RX_PIN = 15;        // GPIO pin for UART RX
    constexpr uint32_t BAUDRATE = 921600; // UART baud rate
}

/* Encoding and Decoding BOS messages *************************************************************/
class BOSMessageCodec
{
public:
    // Convert a float value to a 4-byte array
    static std::array<uint8_t, 4> floatToBytes(float value)
    {
        std::array<uint8_t, 4> bytes{};
        std::memcpy(bytes.data(), &value, sizeof(float));
        return bytes;
    }

    // Convert a 4-byte array back to a float value
    static float bytesToFloat(const std::array<uint8_t, 4> &bytes)
    {
        float value;
        std::memcpy(&value, bytes.data(), sizeof(float));
        return value;
    }

    // Convert a 4-byte array to an integer value
    static int bytesToInt(const std::array<uint8_t, 4> &bytes)
    {
        int value;
        std::memcpy(&value, bytes.data(), sizeof(int));
        return value;
    }

    // Convert a 2-byte array to an unsigned 16-bit integer
    static uint16_t bytesToUint16_t(const std::array<uint8_t, 2> &bytes)
    {
        int value;
        std::memcpy(&value, bytes.data(), sizeof(uint16_t));
        return value;
    }
};

/* Manage LED operations using GPIO ***************************************************************/
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
