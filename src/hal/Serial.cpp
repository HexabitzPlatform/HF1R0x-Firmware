#include "hal/Serial.h"
#include "helper/helper.h"
#include "hexabitz/BOSMessage.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <float.h>
#include <errno.h>

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


#define _SERIALCNF_GET_DATA(v)      		(v >> 8)
#define _SERIALCNF_GET_PAR(v)       		((v >> 4) & 0x0F)
#define _SERIALCNF_GET_STOP(v)      		(v & 0x03)

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


static tcflag_t to_c_tcFlag_t(int serialFlags)
{
	tcflag_t tflags;

	unsigned dataBits = _SERIALCNF_GET_DATA(serialFlags);
	unsigned stopBits = _SERIALCNF_GET_STOP(serialFlags);
	bool parityEnable = _SERIALCNF_GET_PAR(serialFlags) != 0;
	bool partyOdd = _SERIALCNF_GET_PAR(serialFlags) == 1;

	memset(&tflags, 0, sizeof(tcflag_t));

	switch (dataBits) {
	case 5: tflags |= CS5; break;
	case 6: tflags |= CS6; break;
	case 7: tflags |= CS7; break;
	default:
	case 8: tflags |= CS8; break;
	}

	if (stopBits == 2)
		tflags |= CSTOPB;

	if (parityEnable)
		tflags |= PARENB;
	if (partyOdd)
		tflags |= PARODD;

	return tflags;
}

static tcflag_t to_i_tcFlag_t(int serialFlags)
{
	tcflag_t tflags;

	bool parityEnable = _SERIALCNF_GET_PAR(serialFlags) != 0;
	bool partyOdd = _SERIALCNF_GET_PAR(serialFlags) == 1;

	memset(&tflags, 0, sizeof(tcflag_t));
	
	if (parityEnable)
		tflags |= INPCK;

	return tflags;
}

HardwareSerial::HardwareSerial(const char *pathname): fd_(0)
{
	if ((pathname != nullptr) and !open(pathname))
		std::cerr << "Can't open the File" << std::endl;
}

HardwareSerial::~HardwareSerial(void)
{
	if (fd_ > 0)
		close(fd_);
}

bool HardwareSerial::open(const char *pathname)
{
	if (pathname == nullptr)
		return false;
	if (*pathname == '\0')
		return false;
	if (fd_ > 0)
		close(fd_);

	fd_ = ::open(pathname, O_RDWR | O_NOCTTY);
	return fd_ > 0;
}

bool HardwareSerial::isOpen(void) const
{
	return fcntl(fd_, F_GETFD) >= 0;
}
 
bool HardwareSerial::begin(unsigned long baud, unsigned cfg)
{
	if (fd_ <= 0)
		return false;
	if (fcntl(fd_, F_SETFL, O_NONBLOCK))
		return false;

	struct termios attr;
	speed_t speed = (speed_t)toSpeed_t(baud);
	memset(&attr, 0, sizeof(attr));
	
	if (tcgetattr(fd_, &attr))
		return false;
	attr.c_iflag = IGNBRK | to_i_tcFlag_t(cfg);
	attr.c_oflag = 0;
	attr.c_cflag = CREAD | CLOCAL | to_c_tcFlag_t(cfg); // Character Bits, Stop Bits and Parity Bits
	attr.c_lflag = 0;	// Non-Canonical Mode

	attr.c_cc[VTIME] = 0;
	attr.c_cc[VMIN] = 0;

	if (cfsetispeed(&attr, speed))
		return false;
	if (cfsetospeed(&attr, speed))
		return false;
	if (tcsetattr(fd_, TCSANOW, &attr))
		return false;

	struct termios readAttr;
	if (tcgetattr(fd_, &readAttr))
		return false;
	if (memcmp(&attr, &readAttr, sizeof(attr)))
		return false;

	return true;
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