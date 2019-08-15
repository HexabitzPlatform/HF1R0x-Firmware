#include "hal/Serial.h"
#include "helper/helper.h"
#include "hexabitz/BOSMessage.h"

#include "Config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <float.h>
#include <errno.h>

#include <iostream>
#include <thread>
#include <chrono>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/types.h>


/**************** Termios(3) Man Page ****************/
/* struct termios {
 * 		tcflag_t c_iflag;	// Input modes
 *		tcflag_t c_oflag;	// Output modes
 *		tcflag_t c_lflag;	// Local modes
 *		tcflag_t c_cflag;	// Control modes
 *		cc_t c_cc[NCCS];
 * };
 *
 * The termios functions describe a general terminal interface that is
 * provided to control asynchronous communications ports.
 *
 *
 *
 *
 *
 *
 *
 */

// https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c/38318768#38318768

static inline speed_t toSpeed_t(unsigned long baud)
{
	if (baud == 50)
		return B50;
	if (baud == 75)
		return B75;
	if (baud == 110)
		return B110;
	if (baud == 134)
		return B134;
	if (baud == 150)
		return B150;
	if (baud == 200)
		return B200;
	if (baud == 300)
		return B300;
	if (baud == 600)
		return B600;
	if (baud == 1200)
		return B1200;
	if (baud == 1800)
		return B1800;
	if (baud == 2400)
		return B2400;
	if (baud == 4800)
		return B4800;
	if (baud == 9600)
		return B9600;
	if (baud == 19200)
		return B19200;
	if (baud == 38400)
		return B38400;
	if (baud == 57600)
		return B57600;
	if (baud == 115200)
		return B115200;
	if (baud == 230400)
		return B230400;
	if (baud == 921600)
		return B921600;

	return B0;
}

static inline unsigned long toBaudLong(speed_t speed)
{
	if (speed == B50)
		return 50;
	if (speed == B75)
		return 75;
	if (speed == B110)
		return 110;
	if (speed == B134)
		return 134;
	if (speed == B150)
		return 150;
	if (speed == B200)
		return 200;
	if (speed == B300)
		return 300;
	if (speed == B600)
		return 600;
	if (speed == B1200)
		return 1200;
	if (speed == B1800)
		return 1800;
	if (speed == B2400)
		return 2400;
	if (speed == B4800)
		return 4800;
	if (speed == B9600)
		return 9600;
	if (speed == B19200)
		return 19200;
	if (speed == B38400)
		return 38400;
	if (speed == B57600)
		return 57600;
	if (speed == B115200)
		return 115200;
	if (speed == B230400)
		return 230400;

	if (speed == B921600)
		return 921600;

	return 0;
}



HardwareSerial::HardwareSerial(const char *pathname): fd_(0)
{
	fd_ = ::open(pathname, O_RDWR | O_NOCTTY);
	if (fd_ < 0) {
		std::cerr << "Can't open the File" << std::endl;
		// exception throw
	}


}

HardwareSerial::~HardwareSerial(void)
{
	if (fd_ > 0)
		close(fd_);
}

void HardwareSerial::begin(unsigned long baud, uint8_t cfg)
{
	if (fd_ <= 0)
		return;
	if (fcntl(fd_, F_SETFL, O_NONBLOCK))
		return;

	struct termios attr;
	speed_t speed = (speed_t)toSpeed_t(baud);
	memset(&attr, 0, sizeof(attr));
	
	if (tcgetattr(fd_, &attr))
		return;
	attr.c_iflag = IGNBRK | IGNPAR;
	attr.c_oflag = 0;
	attr.c_cflag = CS8 | CREAD | CLOCAL; // Character Bits, Stop Bits and Parity Bits
	attr.c_lflag = 0;	// Non-Canonical Mode

	attr.c_cc[VTIME] = 0;
	attr.c_cc[VMIN] = 0;

	if (cfsetispeed(&attr, speed))
		return;
	if (cfsetospeed(&attr, speed))
		return;
	if (tcsetattr(fd_, TCSANOW, &attr))
		return;

	struct termios readAttr;
	if (tcgetattr(fd_, &readAttr))
		return;
	if (memcmp(&attr, &readAttr, sizeof(attr)))
		return;

	// std::cout << "Successfull" << std::endl;
}

void HardwareSerial::end(void)
{
	if (fd_ > 0)
		close(fd_);
	fd_ = 0;
}

void HardwareSerial::flush(void)
{
	// std::this_thread::sleep_for(std::chrono::milliseconds(2));
	while (tcdrain(fd_));
	// fsync(fd_);
}

size_t HardwareSerial::write(uint8_t c)
{
	size_t written = ::write(fd_, &c, sizeof(c));
	flush();
	return written;
}

int HardwareSerial::available(void)
{
	int in_bytes;
	ioctl(fd_, FIONREAD, &in_bytes);
	return in_bytes;
}

int HardwareSerial::peek(void)
{
	return -1;
}

int HardwareSerial::read(void)
{
	uint8_t byte;
	if (::read(fd_, &byte, sizeof(byte)) <= 0)
		return -1;
	return byte;
}

int HardwareSerial::println(const char *cstr)
{
	int written = 0;
	int len = strlen(cstr);
	for (int i = 0; i < len; i++)
		written += write(cstr[i]);

	written += write('\r');
	written += write('\n');

	return written;
}

std::string HardwareSerial::readLine(void)
{
	std::string str;
	while (1) {
		if (available() <= 0)
			continue;
		char c = read();
		str.push_back(c);
		if (c == '\n')
			break;
	}
	return str;
}


#ifdef USE_TEST_MAIN

#define	CODE_unknown_message							0
#define	CODE_ping										1
#define	CODE_ping_response								2
#define	CODE_IND_on										3
#define	CODE_IND_off									4
#define	CODE_IND_toggle									5

#define	CODE_hi											10
#define	CODE_hi_response								11
#define	CODE_explore_adj								12
#define	CODE_explore_adj_response						13
#define	CODE_port_dir									14
#define	CODE_baudrate									15
#define	CODE_module_id									16
#define	CODE_topology									17
#define	CODE_broadcast_plan								18
#define	CODE_read_port_dir								19
#define	CODE_read_port_dir_response						20
#define	CODE_exp_eeprom	 								21
#define	CODE_def_array	 								22
#define	CODE_CLI_command 								23
#define	CODE_CLI_response  								24
#define	CODE_update  									25
#define	CODE_update_via_port  							26
#define	CODE_DMA_channel  								27
#define	CODE_DMA_scast_stream  							28

#define	CODE_read_remote  								30
#define	CODE_read_remote_response  						31
#define	CODE_write_remote  								32
#define	CODE_write_remote_response  					33
#define	CODE_write_remote_force							34


void exampleTerminal(HardwareSerial& serial)
{
	while (1) {
		std::string str;
		std::cin >> str;
		serial.println(str.c_str());

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		while (serial.available())
			std::cout << char(serial.read());
	}
}

void testBinaryMessage(HardwareSerial& serial)
{
	while (1) {
		hstd::Message m;
		m.setSource(uint8_t(0));
		m.setDest(uint8_t(1));
		m.setCode(uint16_t(CODE_hi));
		m.setMessOnlyFlag(true);
		m.setCLIOnlyFlag(true);
		// m.setTraceFlag(true);
		m.sanitize();

		std::cout << "Sending..." << std::endl;
		std::cout << m << std::endl;
		std::cout << std::string(m) << std::endl;
		hstd::write(serial, m);
		m = hstd::read(serial);
		std::cout << "Received: " << std::endl;
		std::cout << m << std::endl;

		std::this_thread::sleep_for(std::chrono::seconds(2));
	}
}

int main(int argc, char *argv[])
{
	std::cout << "Program Started" << std::endl;
	std::cout << "Major: " << VERSION_MAJOR << std::endl;
	std::cout << "Minor: " << VERSION_MINOR << std::endl;

	if (argc != 2) {
		std::cout << "Two arguments required" << std::endl;
		// exit(EXIT_FAILURE);
	}

	HardwareSerial serial("/dev/ttyUSB0");
	serial.begin(921600);

	testBinaryMessage(serial);

	std::cout << "Closing Program" << std::endl;
	serial.end();

	return 0;
}

#endif