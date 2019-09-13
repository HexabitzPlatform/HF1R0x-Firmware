#include "hexabitz/ProxyModule.h"
#include "helper/helper.h"

#include "hal/Serial.h"
#include "hexabitz/Service.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/BOSMessageBuilder.h"

#include <iostream>
#include <thread>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>
#include <float.h>
#include <errno.h>


ProxyModule::ProxyModule(std::string part, int numPorts, hstd::uid_t uid): id_(uid), numOfPorts_(numPorts), info_(part)
{

}

ProxyModule::ProxyModule(std::string part, int numPorts): id_(hstd::Addr_t::INVALID_UID), numOfPorts_(numPorts), info_(part)
{
	if (numOfPorts_ <= 0)
		numOfPorts_ = BOS::getNumOfPorts(BOS::toPartNumberEnum(part));
}

ProxyModule::ProxyModule(BOS::module_pn_e part): id_(hstd::Addr_t::INVALID_UID), numOfPorts_(-1), info_()
{
	info_ = BOS::toString(part);
	numOfPorts_ = BOS::getNumOfPorts(part);
}

ProxyModule::~ProxyModule(void)
{

}

hstd::uid_t ProxyModule::getUID(void) const
{
	return id_;
}

void ProxyModule::setUID(hstd::uid_t newID)
{
	id_ = newID;
}

int ProxyModule::getNumOfPorts(void) const
{
	return numOfPorts_;
}

bool ProxyModule::isMaster(void) const
{
	return hstd::Addr_t::isMaster(id_);
}

bool ProxyModule::isValidID(void) const
{
	return hstd::Addr_t::isValidUID(id_);
}

void ProxyModule::setNumOfPorts(int ports)
{
	numOfPorts_ = ports;
}

std::string ProxyModule::getPartStr(void) const
{
	return info_;
}

BOS::module_pn_e ProxyModule::getPartNum(void) const
{
	return BOS::toPartNumberEnum(info_);
}

uint16_t ProxyModule::getPartNum_ui16(void) const
{
	return static_cast<uint16_t>(getPartNum());
}

bool ProxyModule::send(const hstd::Message& m)
{
	return Service::getInstance()->send(m);
}

bool ProxyModule::receive(hstd::Message& m, long timeout)
{
	return Service::getInstance()->receive(m, timeout);
}