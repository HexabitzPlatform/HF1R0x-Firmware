#include "Porting.h"           // GPIO + UART hardware abstraction
#include "UARTParser.h"        // BOS byte stream parser
#include "BOS_MessageParser.h" // BOS message payload handler

#include <iostream>
#include <thread>
#include <chrono>

int main()
{
    // Initialize UART on Raspberry Pi UART port (TX=GPIO14, RX=GPIO15, baudrate=115200)
    Porting::initUART(14, 15, 921600);

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

    // Send test packet once (Loopback Test)
    // std::vector<uint8_t> test_packet = {0x48, 0x5A, 0x04, 0x02, 0x01, 0x00, 0x01, 0xE3};
    // Porting::uartSend(std::string(test_packet.begin(), test_packet.end()));

    // Keep main thread alive indefinitely to allow background UART reading thread to run
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
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

// #include <iostream>
// #include <fcntl.h>   // open()
// #include <termios.h> // termios, tcgetattr, tcsetattr
// #include <unistd.h>  // write(), close()
// #include <cstring>   // strerror()

// int main()
// {
//     // const char *uartPort = "/dev/ttyAMA10";

//     const char *uartPort = "/dev/ttyAMA0"; // instead of ttyAMA10

//     int serialFd = open(uartPort, O_RDWR | O_NOCTTY | O_NDELAY);

//     if (serialFd == -1)
//     {
//         std::cerr << "Failed to open " << uartPort << ": " << strerror(errno) << std::endl;
//         return 1;
//     }

//     // Configure UART using termios
//     termios tty{};
//     if (tcgetattr(serialFd, &tty) != 0)
//     {
//         std::cerr << "Error getting termios attributes: " << strerror(errno) << std::endl;
//         close(serialFd);
//         return 1;
//     }

//     // Set baud rate
//     cfsetospeed(&tty, B115200);
//     cfsetispeed(&tty, B115200);

//     // 8N1 Mode: 8 data bits, no parity, 1 stop bit
//     tty.c_cflag &= ~PARENB; // No parity
//     tty.c_cflag &= ~CSTOPB; // 1 stop bit
//     tty.c_cflag &= ~CSIZE;
//     tty.c_cflag |= CS8; // 8 data bits

//     tty.c_cflag &= ~CRTSCTS;       // No hardware flow control
//     tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

//     tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
//     tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
//     tty.c_oflag &= ~OPOST;                          // Raw output

//     tty.c_cc[VMIN] = 1;
//     tty.c_cc[VTIME] = 0;

//     // Apply settings
//     if (tcsetattr(serialFd, TCSANOW, &tty) != 0)
//     {
//         std::cerr << "Error setting termios attributes: " << strerror(errno) << std::endl;
//         close(serialFd);
//         return 1;
//     }

//     // Send test message
//     const char *message = "Hello from Raspberry Pi 5 UART!\r\n";
//     ssize_t bytesWritten = write(serialFd, message, strlen(message));

//     if (bytesWritten < 0)
//     {
//         std::cerr << "Failed to write to UART: " << strerror(errno) << std::endl;
//     }
//     else
//     {
//         std::cout << "Wrote " << bytesWritten << " bytes to UART.\n";
//     }

//     close(serialFd);
//     return 0;
// }
