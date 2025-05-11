// Porting/Porting.cpp
#include "Porting.h"
#include <gpiod.h>
#include <pigpio.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <cstring>  // for std::memcpy
#include <unistd.h> // for sleep

#define CHIP_NAME "gpiochip0"

namespace
{
    const int UART_DEVICE = 0; // pigpio UART handle
    std::atomic<bool> receiving{false};
}

namespace Porting
{

    void initGPIO(int pin)
    {
        // No explicit init needed for libgpiod, just placeholder
        (void)pin;
    }

    void writeGPIO(int pin, bool value)
    {
        gpiod_chip *chip = gpiod_chip_open_by_name(CHIP_NAME);
        if (!chip)
        {
            std::cerr << "Failed to open chip\n";
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
        gpiod_line *line = gpiod_chip_get_line(chip, pin);

        gpiod_line_request_input(line, "BOS");
        int value = gpiod_line_get_value(line);

        gpiod_line_release(line);
        gpiod_chip_close(chip);

        return value == 1;
    }

    /* UART Functions: */

    void initUART(int txPin, int rxPin, int baudrate)
    {
        if (gpioInitialise() < 0)
        {
            std::cerr << "pigpio initialization failed" << std::endl;
            return;
        }
        // Note: pigpio opens serial via gpioSerialOpen with tty name
        // This example uses default /dev/serial0
        // For TX/RX pin configuration, you must use `dtoverlay=uartX` in /boot/config.txt
    }

    void uartSend(const std::string &message)
    {
        int handle = serOpen((char *)"/dev/serial0", 115200, 0);
        if (handle < 0)
        {
            std::cerr << "Failed to open UART device" << std::endl;
            return;
        }

        char *buffer = new char[message.size()];
        std::memcpy(buffer, message.c_str(), message.size());

        serWrite(handle, buffer, message.size());

        delete[] buffer;
        serClose(handle);
    }

    void uartReceive(const std::function<void(char)> &onReceiveChar)
    {
        std::thread([onReceiveChar]()
                    {
            int handle = serOpen((char*)"/dev/serial0", 115200, 0);
            if (handle < 0) {
                std::cerr << "Failed to open UART device" << std::endl;
                return;
            }
    
            receiving = true;
            while (receiving) {
                if (serDataAvailable(handle) > 0) {
                    char c;
                    serRead(handle, &c, 1);
                    onReceiveChar(c);
                } else {
                    gpioDelay(1000); // 1ms delay
                }
            }
            serClose(handle); })
            .detach();
    }

    static std::function<void(char)> uartCallback;

    void setUartReceiveCallback(std::function<void(char)> callback)
    {
        uartCallback = callback;

        std::thread([]
                    {
            int handle = serOpen((char*)"/dev/serial0", 115200, 0);
            if (handle < 0) {
                std::cerr << "Failed to open UART device for receiving\n";
                return;
            }

            while (true) {
                if (serDataAvailable(handle) > 0) {
                    char c;
                    serRead(handle, &c, 1);
                    uartCallback(c);  // Call your callback
                }
                gpioDelay(1000); // 1ms delay
            }

            serClose(handle); })
            .detach();
    }
}
