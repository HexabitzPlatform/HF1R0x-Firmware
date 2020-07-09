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
#include <signal.h>



Service *Service::self_ = nullptr;


Service::Service(void): serial_(nullptr)
{
	info_.reset();
	num_modules_ = 50;

	if (info_.fromBinaryFile(DEF_CACHE_FILENAME))
		std::cout << "Successfully read modules information from " << DEF_CACHE_FILENAME << std::endl;
	else
		std::cout << "Failed to parse " << DEF_CACHE_FILENAME << std::endl;
}

Service::~Service(void)
{
	serial_.end();
}

Service *Service::getInstance(void)
{
	if (self_ == nullptr)
		self_ = new Service();
	if (self_ == nullptr)	// TODO: Handle this. Throw Exception
		return self_;
	return self_;
}

void Service::osDelay(int milliseconds)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}	

int Service::init(const char *pathname)
{
	if (!serial_.open(pathname))
		return -EIO;

	std::cout << "Starting Serial Port" << std::endl;
	serial_.begin(DEFAULT_BAUDRATE);
	return 0;
}

int Service::init(const std::string& pathname)
{
	return init(pathname.c_str());
}

void Service::setOwn(std::shared_ptr<ProxyModule> module)
{
	owner_ = module;
}

std::shared_ptr<ProxyModule> Service::getOwn(void)
{
	return owner_;
}

void Service::enterCriticalSection(void)
{;
	sigset_t mask;
	sigemptyset(&mask);
	sigaddset(&mask, SIGINT);

	if (sigprocmask(SIG_BLOCK, &mask, NULL))
		perror("sigprocmask");
}

void Service::exitCriticalSection(void)
{
	sigset_t mask;
	sigemptyset(&mask);
	sigaddset(&mask, SIGINT);

	if (sigprocmask(SIG_UNBLOCK, &mask, NULL))
		perror("sigprocmask");
}

// TODO: Check Sending
int Service::send(hstd::Message msg)
{
	int ret = 0;
	if (owner_ == nullptr)
		return -EINVAL;
	if (!msg.getSource().hasValidUID())
		msg.getSource().setUID(owner_->getUID());
	if (!msg.getSource().hasValidPort())
		msg.getSource().setPort(hstd::Addr_t::MIN_PORT);
	if (!msg.getDest().hasValidPort())
		msg.getDest().setPort(hstd::Addr_t::MIN_PORT);

	std::cout << "Sending: " << msg << std::endl;
	std::vector<hstd::Frame> list = hstd::buildFramesFromMessage(msg);
	if (list.empty()) {
		std::cout << "Invalid Message/Frame" << std::endl;
		return -EAGAIN;
	}
	enterCriticalSection();
	for (int i = 0; i < list.size(); i++) {
		hstd::Frame& f = list[i];
		std::cout << "Sending (" << i << "th): " << f << std::endl;
		if ((ret = send(f))) break; 
		osDelay(10);
	}
	exitCriticalSection();

	return ret;
}

int Service::send(const hstd::Frame& f)
{
	if (!f.isValid())
		return -EINVAL;
	if (!serial_.isOpen())
		return -EIO;

	BinaryBuffer buffer = f.toBuffer();
	for (int i = 0; i < buffer.getLength(); i++) {
		serial_.write(buffer[i]);
		osDelay(1);
	}

	return 0;
}


int Service::receive(hstd::Message& msg, long timeout)
{
	int result = 0;
	std::vector<hstd::Frame> list;

	using namespace std::chrono;
	auto currentTime_ms = []() -> long {
		return duration_cast<milliseconds>( system_clock::now().time_since_epoch() ).count();
	};

	while (1) {
		hstd::Frame f;
		long start = currentTime_ms();
		if ((result = receive(f, timeout)))
			return result;
		if (timeout >= 0)
			timeout = hstd::constrainLower(timeout - (currentTime_ms() - start), 0L);

		std::cout << "Received: " << f << std::endl;
		list.push_back(f);

		if (hstd::buildMessageFromFrames(list, msg)) {
			std::cout << "Received Message: " << msg << std::endl;
			return 0;
		}
	}
	return -EAGAIN;
}

int Service::receive(hstd::Frame& f, long timeout)
{
	BinaryBuffer buffer;

	using namespace std::chrono;
	auto currentTime_ms = []() {
		return duration_cast<milliseconds>( system_clock::now().time_since_epoch() ).count();
	};

	long start = currentTime_ms();

	while (1) {
		if ((timeout >= 0) and ((currentTime_ms() - start) >= timeout))
			return -ETIMEDOUT;
		if (!serial_.available())
			continue;

		uint8_t c = serial_.read();
		f.param.reset();
		buffer.append(c);
		if (f.fromBuffer(buffer))
			return 0;
	}
	return -EAGAIN;
}


std::vector<hstd::Addr_t> Service::FindRoute(hstd::Addr_t dest)
{
	return info_.FindRoute(dest.getUID(), owner_->getUID());		
}

std::vector<hstd::Addr_t> Service::FindRoute(hstd::uid_t destID)
{
	return info_.FindRoute(destID, owner_->getUID());		
}

hstd::port_t Service::FindSrcPortFor(hstd::uid_t destID)
{
	std::vector<hstd::Addr_t> route = FindRoute(destID);
	if (route.size() <= 0)
		return hstd::Addr_t::INVALID_PORT;
	return route.at(0).getPort();
}


int Service::ping(hstd::uid_t destID)
{
	int ret = 0;
	hstd::Message msg = hstd::make_ping_message(destID);

	if ((ret = send(msg)))
		return ret;
	if ((ret = receive(msg)))
		return ret;
	if (msg.getCode() != CODE_ping_response)
		return -EAGAIN;
	return 0;
}

int Service::ping(hstd::uid_t destID, hstd::uid_t srcID)
{
	int ret = 0;
	hstd::Message msg = hstd::make_ping_message(destID, srcID);

	if ((ret = send(msg)))
		return ret;
	if ((ret = receive(msg)))
		return ret;
	if (msg.getCode() != CODE_ping_response)
		return -EAGAIN;
	return 0;
}


int Service::assignIDToNeigh(hstd::uid_t id, hstd::port_t portNum)
{
	int ret = 0;
	hstd::Message msg = hstd::make_message_meighbour(portNum, CODE_module_id);

	msg.getParams().append(uint8_t(0));	/* Change own ID */
	msg.getParams().append(uint8_t(id));
	msg.getParams().append(uint8_t(0));

	if ((ret = send(msg)))
		return ret;
	// No response to this message. So Add Delay
	osDelay(50);
	return 0;
}

int Service::assignIDToAdjacent(hstd::uid_t destID, hstd::port_t port, hstd::uid_t newID)
{
	int ret = 0;
	hstd::Message msg = hstd::make_message(destID, CODE_module_id);
	msg.getParams().append(uint8_t(1));	/* Change own ID */
	msg.getParams().append(uint8_t(newID));
	msg.getParams().append(uint8_t(port));

	if ((ret = send(msg)))
		return ret;
	// No Response to this Message
	osDelay(50);
	return ret;	
}

int Service::sayHiToNeighbour(hstd::port_t port, enum BOS::module_pn_e& part, hstd::Addr_t& neigh)
{
	int ret = 0;
	/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
	hstd::Message msg = hstd::make_message(hstd::Addr_t(0, 0), hstd::Addr_t(getOwn()->getUID(), port), CODE_hi);
	enum BOS::module_pn_e ownPart = getOwn()->getPartNum();

	msg.getParams().append(uint8_t(highByte(ownPart)));
	msg.getParams().append(uint8_t(lowByte(ownPart)));
	msg.getParams().append(uint8_t(port));
	
	if ((ret = send(msg)))
		return ret;
	if ((ret = receive(msg)))
		return ret;
	if (msg.getCode() != CODE_hi_response)
		return -EAGAIN;

	/* Neighbor PN */
	uint8_t partNumMSB = msg.getParams().popui8();
	uint8_t partNumLSB = msg.getParams().popui8();
	part = static_cast<enum BOS::module_pn_e>((uint16_t(partNumMSB) << 8) | partNumLSB);
	/* Neighbor ID + Neighbor own port */
	neigh.setUID(msg.getSource().getUID());
	neigh.setPort(msg.getParams().popui8());
	return 0;
}

int Service::ExploreNeighbors(hstd::port_t ignore, NeighboursInfo& info)
{
	int ret = 0;
	const int NUM_PORTS = getOwn()->getNumOfPorts();

	/* Send Hi messages to adjacent neighbors */
	for (hstd::port_t port = 1; port <= NUM_PORTS; port++) {
		hstd::Addr_t neigh;
		enum BOS::module_pn_e part;

		if (port == ignore)
			continue;
		if ((ret = sayHiToNeighbour(port, part, neigh)))
			break;

		// std::cout << "Hi Neigh: " << neigh.getUID() << " " << neigh.getPort() << std::endl;
		info.setAddrInfoFor(port, neigh);
		info.setPartInfoFor(port, part);
	}
	return ret;
}

int Service::ExploreAdjacentOf(hstd::Addr_t addr, NeighboursInfo& info)
{
	int ret = 0;
	hstd::Message msg = hstd::make_message(addr, CODE_explore_adj);
	if ((ret = send(msg)))
		return ret;
	if ((ret = receive(msg)))
		return ret;

	if (msg.getCode() != CODE_explore_adj_response)
		return -EAGAIN;

	info.reset();
	info.fromBinaryBuffer(msg.getParams());
	return ret;
}

void Service::changePortDir(hstd::port_t port, enum BOS::PortDir dir)
{
	// We don't have anything to do in hardware
	// We can change the direction of the port
	info_.setPortDir(getOwn()->getUID(), port, dir);
	if (dir == BOS::PortDir::REVERSED)
		return;
	return;
}


int Service::syncTopologyTo(hstd::uid_t destID)
{
	int ret = 0;
	hstd::Message msg = hstd::make_message(hstd::Addr_t(destID), CODE_topology);
	BinaryBuffer buffer = info_.toBinaryBuffer(num_modules_);
	msg.getParams().append(buffer);

	std::cout << "<<<< Syncing Modules Information to all nodes >>>>" << std::endl;
	std::cout  << std::endl << info_.toString(num_modules_) << std::endl;
	if ((ret = send(msg)))
		return ret;

	// No response to this message. So Add Delay
	osDelay(100);
	return ret;
}

int Service::synPortDir(hstd::uid_t dest)
{
	int ret = 0;
	hstd::Message msg = hstd::make_message(dest, CODE_port_dir);

	for (hstd::port_t p = 1 ; p <= BOS::MAX_NUM_OF_PORTS; p++) {

		hstd::Addr_t addr = hstd::Addr_t(dest, p);
		hstd::Addr_t nAddr = info_.getModuleConnAt(addr);

		if (!info_.hasConnInfo(addr) or info_.isPortDirReversed(nAddr))	{	
			/* If empty port leave BOS::PortDir::NORMAL */
			msg.getParams().append(uint8_t(BOS::PortDir::NORMAL));
			info_.setPortDirNormal(addr);
		} else {
			msg.getParams().append(uint8_t(BOS::PortDir::REVERSED));
			info_.setPortDirReversed(addr);			
		}
	}
	
	/* Step 5c - Check if an inport is BOS::PortDir::REVERSED */
	/* Find out the inport to this module from master */
	std::vector<hstd::Addr_t> route = info_.FindRoute(hstd::Addr_t(dest), hstd::Addr_t(1));
	if (!route.size())
		return -EINVAL;
	hstd::uid_t justNextMod = route[route.size() - 1].getUID();
	hstd::port_t portOut = route[route.size() - 1].getPort();

	/* Is the inport BOS::PortDir::REVERSED? */
	if (msg.getParams()[portOut - 1] == uint8_t(BOS::PortDir::REVERSED))
		msg.getParams().append(uint8_t(BOS::PortDir::REVERSED));		/* Make sure the inport is BOS::PortDir::REVERSED */
	else
		msg.getParams().append(uint8_t(BOS::PortDir::NORMAL));

	if ((ret = send(msg)))
		return ret;

	osDelay(10);
	return 0;
}

int Service::broadcastToSave(void)
{
	int ret = 0;
	hstd::Message msg = hstd::make_broadcast(CODE_exp_eeprom);
	if ((ret = send(msg)))
		return ret;
	osDelay(100);
	return 0;
}

int Service::reverseAllButInPort(hstd::uid_t destID)
{
	int ret = 0;
	hstd::Message msg = hstd::make_message(hstd::Addr_t(destID), CODE_port_dir);

	for (hstd::port_t p = 1 ; p <= BOS::MAX_NUM_OF_PORTS; p++)
		msg.getParams().append(uint8_t(BOS::PortDir::REVERSED));
	msg.getParams().append(uint8_t(BOS::PortDir::NORMAL)); /* Make sure the inport is not BOS::PortDir::REVERSED */

	if ((ret = send(msg)))
		return ret;
	// No Response for this Message. Add a Delay
	osDelay(10);
	return 0;
}
