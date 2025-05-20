#include "Porting.h"
#include <gpiod.h> /* For GPIO control using libgpiod */
#include <thread>  /* For std::thread to handle UART receive asynchronously */
#include <atomic>
#include <iostream>  /* For std::cerr and std::cout */
#include <cstring>   /* For strerror() */
#include <unistd.h>  /* For sleep, read, write, close */
#include <fcntl.h>   /* For file control options, like O_RDWR */
#include <termios.h> /* For UART configuration using POSIX APIs */

#define CHIP_NAME "gpiochip0" /* Default GPIO chip on Raspberry Pi */

namespace
{
    constexpr const char *UART_DEVICE_PATH = "/dev/ttyAMA0"; /* Device path for UART */
    int uartFd = -1;                                         /* POSIX file descriptor for UART. Initialized to invalid value (-1). */
}

namespace Porting
{
    /* ========== GPIO Functions ========== */

    void initGPIO(int pin)
    {
        /* Placeholder function for GPIO initialization */
        (void)pin; /* Suppress unused parameter warning */
    }

    void writeGPIO(int pin, bool value)
    {
        /* Opens the GPIO chip and configures the given pin as output, then sets its value */

        gpiod_chip *chip = gpiod_chip_open_by_name(CHIP_NAME); /* Open default GPIO chip */
        if (!chip)
        {
            std::cerr << "Failed to open GPIO chip\n";
            return;
        }

        gpiod_line *line = gpiod_chip_get_line(chip, pin); /* Get specific GPIO line by pin number */
        if (!line)
        {
            std::cerr << "Failed to get GPIO line\n";
            gpiod_chip_close(chip);
            return;
        }

        /* Request the GPIO line for output */
        if (gpiod_line_request_output(line, "BOS", 0) < 0)
        {
            std::cerr << "Failed to request line as output\n";
            gpiod_chip_close(chip);
            return;
        }

        gpiod_line_set_value(line, value ? 1 : 0); /* Set the output value: 1 = High, 0 = Low */

        gpiod_line_release(line); /* Release line after use */
        gpiod_chip_close(chip);   /* Close GPIO chip */
    }

    bool readGPIO(int pin)
    {
        /* Opens the GPIO chip, sets the pin as input, and reads its value */

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

        /* Request the GPIO line for input */
        if (gpiod_line_request_input(line, "BOS") < 0)
        {
            std::cerr << "Failed to request line as input\n";
            gpiod_chip_close(chip);
            return false;
        }

        int value = gpiod_line_get_value(line); /* Read the GPIO input value */

        gpiod_line_release(line);
        gpiod_chip_close(chip);

        return value == 1; /* Return true if pin is high */
    }

    /* ========== UART Functions (POSIX) ========== */

    void initUART(int txPin, int rxPin, int baudrate)
    {
        /* Initialize UART using POSIX APIs and configure baud rate, data bits, and parity */
        (void)txPin;
        (void)rxPin;
        (void)baudrate;

        uartFd = open(UART_DEVICE_PATH, O_RDWR | O_NOCTTY | O_NDELAY); /* Open UART device */
        if (uartFd == -1)
        {
            std::cerr << "Failed to open UART device: " << strerror(errno) << std::endl;
            return;
        }

        termios tty{}; /* termios struct holds UART settings */
        if (tcgetattr(uartFd, &tty) != 0)
        {
            std::cerr << "Error getting UART attributes: " << strerror(errno) << std::endl;
            close(uartFd);
            uartFd = -1;
            return;
        }

        /* Set baud rate to 921600 */
        cfsetospeed(&tty, B921600);
        cfsetispeed(&tty, B921600);

        /* 8N1 configuration: 8 data bits, no parity, 1 stop bit */
        tty.c_cflag &= ~PARENB; /* No parity */
        tty.c_cflag &= ~CSTOPB; /* 1 stop bit */
        tty.c_cflag &= ~CSIZE;  /* Clear current char size mask */
        tty.c_cflag |= CS8;     /* 8 data bits */

        tty.c_cflag &= ~CRTSCTS;       /* Disable hardware flow control */
        tty.c_cflag |= CREAD | CLOCAL; /* Enable receiver, ignore modem control lines */

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /* Raw input mode */
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);         /* Disable software flow control */
        tty.c_oflag &= ~OPOST;                          /* Raw output mode */

        tty.c_cc[VMIN] = 1;  /* Minimum number of characters to read */
        tty.c_cc[VTIME] = 0; /* Timeout */

        if (tcsetattr(uartFd, TCSANOW, &tty) != 0)
        {
            std::cerr << "Error setting UART attributes: " << strerror(errno) << std::endl;
            close(uartFd);
            uartFd = -1;
        }
    }

    void uartSend(const std::vector<uint8_t> &data)
    {
        if (uartFd == -1)
        {
            std::cerr << "UART not initialized or failed to open.\n";
            return;
        }

        ssize_t bytesWritten = write(uartFd, data.data(), data.size());

        if (bytesWritten < 0)
        {
            std::cerr << "UART write failed: " << strerror(errno) << std::endl;
        }
    }

    void uartReceive(const std::function<void(char)> &onReceiveChar)
    {
        /* Spawns a thread that continuously reads 1 byte from UART and calls the callback function */

        if (uartFd == -1)
        {
            std::cerr << "UART not initialized or failed to open.\n";
            return;
        }

        /* Launch background thread using C++11 lambda and std::thread */
        std::thread([onReceiveChar]()
                    {
            char c;
            while (true)
            {
                ssize_t bytesRead = read(uartFd, &c, 1); /* Read 1 byte at a time */
                if (bytesRead == 1)
                {
                    std::cout << "Received byte: 0x" << std::hex << (int)c << std::endl;
                    onReceiveChar(c); /* Call user-defined callback with received char */
                }

                usleep(1000); /* Sleep 1ms to reduce CPU usage */
            } })
            .detach(); /* Detach thread so it runs independently */
    }

    /* Global static function pointer to store the callback */
    static std::function<void(char)> uartCallback;

    void setUartReceiveCallback(std::function<void(char)> callback)
    {
        /* Sets the UART receive callback function using std::function */
        uartCallback = std::move(callback); /* Move semantics avoids unnecessary copying */

        uartReceive([](char c)
                    {
            if (uartCallback)
            {
                uartCallback(c); /* Call stored callback if set */
            } });
    }

} /* namespace Porting */
