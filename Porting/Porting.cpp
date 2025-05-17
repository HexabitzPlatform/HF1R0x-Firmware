#include "Porting.h"
#include <gpiod.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <cstring>
#include <unistd.h>  // for sleep, read, write, close
#include <fcntl.h>   // for open
#include <termios.h> // for termios

#define CHIP_NAME "gpiochip0"

namespace
{
    constexpr const char *UART_DEVICE_PATH = "/dev/ttyAMA0";
    int uartFd = -1; // POSIX UART file descriptor
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

    // ========== UART Functions (POSIX) ==========

    void initUART(int txPin, int rxPin, int baudrate)
    {
        (void)txPin;
        (void)rxPin;

        uartFd = open(UART_DEVICE_PATH, O_RDWR | O_NOCTTY | O_NDELAY);
        if (uartFd == -1)
        {
            std::cerr << "Failed to open UART device: " << strerror(errno) << std::endl;
            return;
        }

        termios tty{};
        if (tcgetattr(uartFd, &tty) != 0)
        {
            std::cerr << "Error getting UART attributes: " << strerror(errno) << std::endl;
            close(uartFd);
            uartFd = -1;
            return;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(uartFd, TCSANOW, &tty) != 0)
        {
            std::cerr << "Error setting UART attributes: " << strerror(errno) << std::endl;
            close(uartFd);
            uartFd = -1;
        }
    }

    void uartSend(const std::string &message)
    {
        if (uartFd == -1)
        {
            std::cerr << "UART not initialized or failed to open.\n";
            return;
        }

        ssize_t bytesWritten = write(uartFd, message.c_str(), message.size());

        if (bytesWritten < 0)
        {
            std::cerr << "UART write failed: " << strerror(errno) << std::endl;
        }
    }

    void uartReceive(const std::function<void(char)> &onReceiveChar)
    {
        if (uartFd == -1)
        {
            std::cerr << "UART not initialized or failed to open.\n";
            return;
        }

        std::thread([onReceiveChar]()
                    {
            char c;
            while (true)
            {
                ssize_t bytesRead = read(uartFd, &c, 1);
                if (bytesRead == 1)
                {
                    onReceiveChar(c);
                }
                usleep(1000);
            } })
            .detach();
    }

    static std::function<void(char)> uartCallback;

    void setUartReceiveCallback(std::function<void(char)> callback)
    {
        uartCallback = std::move(callback);

        uartReceive([](char c)
                    {
            if (uartCallback)
            {
                uartCallback(c);
            } });
    }

} // namespace Porting
