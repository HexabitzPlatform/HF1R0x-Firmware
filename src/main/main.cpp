#include "hexabitz/Module.h"
#include "helper/helper.h"

#include "hal/Serial.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/BOSMessageBuilder.h"

#include "Config.h"

#include <iostream>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <float.h>
#include <errno.h>


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

void testBinaryMessage(void)
{
	hstd::setCLIRespDefault(true);
	hstd::setTraceDefault(true);

	while (1) {
		hstd::Message m = hstd::make_message(hstd::Addr_t(1,1), hstd::Addr_t(0,1), CODE_hi);

		std::cout << "Sending: " << m << std::endl;
		Service::getInstance()->send(m);

		if (Service::getInstance()->receive(m))
			std::cout << "Received: " << m << std::endl;

		std::this_thread::sleep_for(std::chrono::seconds(2));
	}
}

int main(int argc, char *argv[])
{
	std::cout << "Program Started (";
	std::cout << "Major: " << VERSION_MAJOR << " ";
	std::cout << "Minor: " << VERSION_MINOR << ")" << std::endl;

	Service::getInstance()->init("/dev/ttyUSB1");

	testBinaryMessage();

	std::cout << "Closing Program" << std::endl;
	return 0;
}