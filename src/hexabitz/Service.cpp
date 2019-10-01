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
	for (int i = 0; i < list.size(); i++) {
		hstd::Frame& f = list[i];
		std::cout << "Sending (" << i << "th): " << f << std::endl;
		if ((ret = send(f))) return ret;
	}

	return 0;
}

int Service::send(const hstd::Frame& f)
{
	if (!f.isValid())
		return -EINVAL;

	BinaryBuffer buffer = f.toBuffer();
	for (int i = 0; i < buffer.getLength(); i++) 
		serial_.write(buffer[i]);

	return 0;
}


int Service::receive(hstd::Message& msg, long timeout)
{
	std::vector<hstd::Frame> list;
	while (1) {
		hstd::Frame f;
		if (!receive(f, timeout))
			return false;

		std::cout << "Received: " << f << std::endl;
		list.push_back(f);

		if (hstd::buildMessageFromFrames(list, msg))
			return 0;
	}
	return -EAGAIN;
}

int Service::receive(hstd::Frame& f, long timeout)
{
	BinaryBuffer buffer;

	while (1) {
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


int Service::ping(uint8_t destID)
{
	int ret = 0;
	hstd::Message msg = hstd::make_message(destID, CODE_ping);

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
	osDelay(10);
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
	osDelay(10);
	return ret;	
}

int Service::sayHiToNeighbour(hstd::port_t port, enum BOS::module_pn_e& part, hstd::Addr_t& neigh)
{
	int ret = 0;
	/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
	hstd::Message msg = hstd::make_message_meighbour(port, CODE_hi);

	msg.getParams().append(static_cast<uint16_t>(getOwn()->getPartNum()));
	msg.getParams().append(uint8_t(port));
	
	if ((ret = send(msg)))
		return ret;
	if ((ret = receive(msg)))
		return ret;
	if (msg.getCode() != CODE_hi_response)
		return -EAGAIN;

	/* Neighbor PN */
	part = static_cast<enum BOS::module_pn_e>(msg.getParams().popui16());
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

	if ((ret = send(msg)))
		return ret;

	// No response to this message. So Add Delay
	osDelay(60);
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
	std::vector<hstd::Addr_t> route = info_.FindRoute(hstd::Addr_t(1), hstd::Addr_t(dest));
	hstd::uid_t justNextMod = route[1].getUID();				/* previous module = route[Number of hops - 1] */
	hstd::port_t portOut = info_.FindSourcePort(dest, justNextMod);

	/* Is the inport BOS::PortDir::REVERSED? */
	if ((justNextMod == dest) or (msg.getParams()[portOut - 1] == uint8_t(BOS::PortDir::REVERSED)) )
		msg.getParams().append(uint8_t(BOS::PortDir::REVERSED));		/* Make sure the inport is BOS::PortDir::REVERSED */
	else
		msg.getParams().append(uint8_t(0));

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



int Service::Explore(void)
{
	int result = 0;
	hstd::port_t PcPort = 1;
	NeighboursInfo neighInfo;

	std::shared_ptr<ProxyModule> master = getOwn();
	if (master == nullptr)
		return -EINVAL;
	
	master->setUID(hstd::Addr_t::MASTER_UID);
	info_.setPartNumOf(*master);

	hstd::uid_t myID = master->getUID();
	const int NUM_OF_PORTS = master->getNumOfPorts();

	hstd::uid_t lastID = 0;
	hstd::uid_t currentID = myID;
	
	/* >>> Step 1 - Reverse master ports and explore adjacent neighbors */
	for (hstd::port_t port = 1; port <= NUM_OF_PORTS; port++) {
		if (port != PcPort) changePortDir(port, BOS::PortDir::REVERSED);
	}
	ExploreNeighbors(PcPort, neighInfo);

	
	/* >>> Step 2 - Assign IDs to new modules & update the topology array */
	for (hstd::port_t port = 1; port <= NUM_OF_PORTS; port++) {
		if (!neighInfo.hasInfo(port))
			continue;

		/* Step 2a - Assign IDs to new modules */
		if (assignIDToNeigh(++currentID, port))
			continue;

		neighInfo.setUIDInfoFor(port, currentID);

		hstd::Addr_t ownAddr = hstd::Addr_t(myID, port);
		hstd::Addr_t neighAddr = neighInfo.getAddrAt(port);
		enum BOS::module_pn_e neighPart = neighInfo.getPartEnumAt(port);

		/* Step 2b - Update master topology array */	
		info_.addConnection(ownAddr, neighAddr);
		info_.setPartNumOf(neighAddr, neighPart);

		num_modules_ = currentID;
	}
	
	/* Step 2c - Ask neighbors to update their topology array */
	for (hstd::uid_t i = 2; i <= currentID; i++)
		syncTopologyTo(i);
	
	
	/* >>> Step 3 - Ask each new module to explore and repeat */
	
	while (lastID != currentID) {
		/* Update lastID */
		lastID = currentID;
		
		/* Scan all discovered modules */
		for (hstd::uid_t i = 2 ; i <= currentID; i++) {
			/* Step 3a - Ask the module to reverse ports */
			reverseAllButInPort(i);
			osDelay(10);
			
			/* Step 3b - Ask the module to explore adjacent neighbors */
			NeighboursInfo adjInfo;
			ExploreAdjacentOf(hstd::Addr_t(i), adjInfo);	
		
			for (hstd::port_t p = 1; p <= BOS::MAX_NUM_OF_PORTS; p++) {
				if (!adjInfo.hasInfo(p))
					continue;

				/* Step 3c - Assign IDs to new modules */
				assignIDToAdjacent(i, p, ++currentID);
				adjInfo.setUIDInfoFor(p, currentID);

				hstd::Addr_t ithAddr = hstd::Addr_t(i, p);
				hstd::Addr_t newAddr = adjInfo.getAddrAt(p);
				enum BOS::module_pn_e neighPart = adjInfo.getPartEnumAt(p);

				/* Step 3d - Update master topology array */
				if (newAddr.getUID() == hstd::Addr_t::MASTER_UID)
					continue;
				info_.addConnection(ithAddr, newAddr);
				info_.setPartNumOf(newAddr, neighPart);

				num_modules_ = currentID;
				osDelay(10);
			}

			/* Step 3e - Ask all discovered modules to update their topology array */
			for (hstd::uid_t j = 2; j <= currentID; j++)
				syncTopologyTo(j);
		}	
	}

	
	/* >>> Step 4 - Make sure all connected modules have been discovered */
	
	ExploreNeighbors(PcPort, neighInfo);
	/* Check for any unIDed neighbors */
	if (neighInfo.hasAllIDedInfo()) {
		result = -EINVAL;
		goto END;
	}

	/* Ask other modules for any unIDed neighbors */
	for (hstd::uid_t i = 2; i <= currentID; i++) {
		NeighboursInfo adjInfo;
		ExploreAdjacentOf(hstd::Addr_t(i), adjInfo);

		if (adjInfo.hasAllIDedInfo())
			result = -EAGAIN;			
	}
	
	
	/* >>> Step 5 - If no unIDed modules found, generate and distribute port directions */
	if (result)
		goto END;

	/* Step 5a - Virtually reset the state of master ports to BOS::PortDir::NORMAL */
	for (hstd::port_t p = 1; p <= NUM_OF_PORTS; p++)
		info_.setPortDirNormal(master->getUID(), p);
	
	/* Step 5b - Update other modules ports starting from the last one */
	for (hstd::uid_t i = currentID; i >= 2; i--) {
		if (synPortDir(i))
			continue;		
	}			

	/* Step 5e - Update master ports > all BOS::PortDir::NORMAL */
	
			
	/* >>> Step 6 - Test new port directions by pinging all modules */
	
	if (result)
		goto END; 

	Service::osDelay(100);
	// BOS.response = BOS_RESPONSE_MSG;		// Enable response for pings
	for (hstd::uid_t i = 2; i <= num_modules_; i++) {
		if (ping(i)) {
			result = -EINVAL;
			break;
		}
	}
	
	/* >>> Step 7 - Save all (topology and port directions) in RO/EEPROM */
	if (result)
		goto END;
	/* Save data in the master */
	info_.toBinaryFile(DEF_CACHE_FILENAME);
	osDelay(100);
	/* Ask other modules to save their data too */
	broadcastToSave();
	

END:
	return result;
}
