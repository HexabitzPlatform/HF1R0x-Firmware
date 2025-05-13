#include "Porting.h"
#include <gpiod.h>
#include <pigpio.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <cstring>
#include <unistd.h> // for sleep

#define CHIP_NAME "gpiochip0"

// UART configuration constants
namespace
{
    constexpr const char *UART_DEVICE_PATH = "/dev/serial0";
    constexpr int UART_BAUDRATE = 115200;
}

namespace Porting
{
    // ========== GPIO Functions ==========

    void initGPIO(int pin)
    {
        // Placeholder for any GPIO init logic, depending on the system
        (void)pin;
    }

    void writeGPIO(int pin, bool value)
    {
        gpiod_chip *chip = gpiod_chip_open_by_name(CHIP_NAME);
        if (!chip)
        {
            std::cerr << "Failed to open GPIO chip\n";
            return;
        }

        gpiod_line *line = gpiod_chip_get_line(chip, pin);
        if (!line)
        {
            std::cerr << "Failed to get GPIO line\n";
            gpiod_chip_close(chip);
            return;
        }

        if (gpiod_line_request_output(line, "BOS", 0) < 0)
        {
            std::cerr << "Failed to request line as output\n";
            gpiod_chip_close(chip);
            return;
        }

        gpiod_line_set_value(line, value ? 1 : 0);

        gpiod_line_release(line);
        gpiod_chip_close(chip);
    }

    bool readGPIO(int pin)
    {
        gpiod_chip *chip = gpiod_chip_open_by_name(CHIP_NAME);
        if (!chip)
        {
            std::cerr << "Failed to open GPIO chip\n";
            return false;
        }

        gpiod_line *line = gpiod_chip_get_line(chip, pin);
        if (!line)
        {
            std::cerr << "Failed to get GPIO line\n";
            gpiod_chip_close(chip);
            return false;
        }

        if (gpiod_line_request_input(line, "BOS") < 0)
        {
            std::cerr << "Failed to request line as input\n";
            gpiod_chip_close(chip);
            return false;
        }

        int value = gpiod_line_get_value(line);

        gpiod_line_release(line);
        gpiod_chip_close(chip);

        return value == 1;
    }

    // ========== UART Functions ==========

    void initUART(int txPin, int rxPin, int baudrate)
    {
        (void)txPin;
        (void)rxPin;
        (void)baudrate;

        if (gpioInitialise() < 0)
        {
            std::cerr << "pigpio initialization failed" << std::endl;
        }

        // TX/RX pin remapping is done via /boot/config.txt overlays on Pi.
    }

    void uartSend(const std::string &message)
    {
        int handle = serOpen(const_cast<char *>(UART_DEVICE_PATH), UART_BAUDRATE, 0);

        if (handle < 0)
        {
            std::cerr << "Failed to open UART device for sending\n";
            return;
        }

        serWrite(handle, const_cast<char *>(message.c_str()), message.size());
        serClose(handle);
    }

    void uartReceive(const std::function<void(char)> &onReceiveChar)
    {
        std::thread([onReceiveChar]()
                    {
                        int handle = serOpen(const_cast<char*>(UART_DEVICE_PATH), UART_BAUDRATE, 0);

            if (handle < 0) {
                std::cerr << "Failed to open UART device for receiving\n";
                return;
            }

            while (true) {
                if (serDataAvailable(handle) > 0) {
                    char c;
                    if (serRead(handle, &c, 1) == 1) {
                        onReceiveChar(c);
                    }
                }
                gpioDelay(1000); // 1 ms delay
            }

            serClose(handle); })
            .detach();
    }

    static std::function<void(char)> uartCallback;

    void setUartReceiveCallback(std::function<void(char)> callback)
    {
        uartCallback = std::move(callback);

        std::thread([]()
                    {
                        int handle = serOpen(const_cast<char*>(UART_DEVICE_PATH), UART_BAUDRATE, 0);

            if (handle < 0) {
                std::cerr << "Failed to open UART device\n";
                return;
            }

            while (true) {
                if (serDataAvailable(handle) > 0) {
                    char c;
                    if (serRead(handle, &c, 1) == 1 && uartCallback) {
                        uartCallback(c);
                    }
                }
                gpioDelay(1000); // 1 ms delay
            }

            serClose(handle); })
            .detach();
    }

    // ========== UART Byte Receive Function ==========

    // int uartReceiveByte() // Call with Porting:: namespace
    // {
    //     int handle = serOpen(const_cast<char *>(UART_DEVICE_PATH), UART_BAUDRATE, 0);
    //     if (handle < 0)
    //     {
    //         std::cerr << "Failed to open UART device for receiving\n";
    //         return -1;
    //     }

    //     char byte;
    //     if (serDataAvailable(handle) > 0)
    //     {
    //         if (serRead(handle, &byte, 1) == 1)
    //         {
    //             serClose(handle);
    //             return static_cast<unsigned char>(byte);
    //         }
    //     }

    //     serClose(handle);
    //     return -1; // Return -1 if no data available
    // }

}