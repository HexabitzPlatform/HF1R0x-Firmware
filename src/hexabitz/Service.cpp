#include "hexabitz/Service.h"

#include "hexabitz/ProxyModule.h"

#include <iostream>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <float.h>
#include <errno.h>


Service *Service::self_ = nullptr;
uint8_t Service::myID = 0;

enum BOS::module_pn_e Service::partNumber = BOS::H01R0;

uint8_t Service::numModules = 50;
uint16_t Service::neighbors[BOS::MAX_NUM_OF_PORTS][2] = { { 0 } };
uint16_t Service::neighbors2[BOS::MAX_NUM_OF_PORTS][2] = { { 0 } };
uint16_t Service::array[BOS::MAX_NUM_OF_MODULES][BOS::MAX_NUM_OF_PORTS + 1] = { { 0 } };
uint16_t Service::arrayPortsDir[BOS::MAX_NUM_OF_MODULES] = { 0 };
uint8_t Service::route[BOS::MAX_NUM_OF_MODULES] = { 0 };
uint8_t Service::routeDist[BOS::MAX_NUM_OF_MODULES] = { 0 };

Service *Service::getInstance(void)
{
	if (self_ == nullptr)
		self_ = new Service();
	if (self_ == nullptr)	// TODO: Throw Exception
		return self_;
	return self_;
}

void Service::osDelay(int milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}	

bool Service::init(const char *pathname)
{
	if (!serial_.open(pathname))
		return false;
	std::cout << "Starting Serial Port" << std::endl;
	serial_.begin(921600);
	return true;
}


int Service::setProxy(ProxyModule *owner)
{
	module_ = std::shared_ptr<ProxyModule>(owner);
	return 0;
}

BOS::PortDir Service::getPortDir(uint8_t id, uint8_t port)
{
	if (arrayPortsDir[id - 1] & (0x8000 >> (port - 1)))
		return BOS::PortDir::REVERSED;
	return BOS::PortDir::NORMAL;
}

void Service::setPortDir(uint8_t id, uint8_t port, BOS::PortDir dir)
{
	if (dir == BOS::PortDir::REVERSED)
		arrayPortsDir[id - 1] |= (0x8000 >> (port - 1));
	else
		arrayPortsDir[id - 1] &= (~(0x8000 >> (port - 1)));	

}


bool Service::send(const hstd::Message& msg)
{
	std::vector<hstd::Frame> list = hstd::buildFramesFromMessage(msg);

	for (int i = 0; i < list.size(); i++) {
		hstd::Frame& f = list[i];
		std::cout << "Sending (" << i << "th): " << f << std::endl;
		if (!send(f)) return false;
	}

	return true;
}

bool Service::send(const hstd::Frame& f)
{
	if (!f.isValid())
		return false;

	BinaryBuffer buffer = f.toBuffer();
	for (int i = 0; i < buffer.getLength(); i++) 
		serial_.write(buffer[i]);

	return true;
}


bool Service::receive(hstd::Message& msg, long timeout)
{
	std::vector<hstd::Frame> list;
	while (1) {
		hstd::Frame f;
		if (!receive(f, timeout))
			return false;

		std::cout << "Received: " << f << std::endl;
		list.push_back(f);

		if (hstd::buildMessageFromFrames(list, msg))
			return true;
	}
	return false;
}

bool Service::receive(hstd::Frame& f, long timeout)
{
	BinaryBuffer buffer;

	while (1) {
		if (!serial_.available())
			continue;

		uint8_t c = serial_.read();
		f.param.reset();
		buffer.append(c);
		if (f.fromBuffer(buffer))
			return true;
	}
	return false;
}
