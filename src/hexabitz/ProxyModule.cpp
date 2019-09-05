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


ProxyModule::ProxyModule(hstd::uid_t idIn, int ports, std::string partStr): id_(idIn), numOfPorts_(ports), info_(partStr)
{

}

ProxyModule::ProxyModule(std::string partStr, int ports): id_(hstd::Addr_t::INVALID_UID), numOfPorts_(ports), info_(partStr)
{

}

ProxyModule::ProxyModule(BOS::module_pn_e partNum, int ports): id_(hstd::Addr_t::INVALID_UID), numOfPorts_(ports), info_()
{
	info_ = BOS::toString(partNum);
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

bool ProxyModule::send(const hstd::Message& m)
{
	return Service::getInstance()->send(m);
}

bool ProxyModule::receive(hstd::Message& m, long timeout)
{
	return Service::getInstance()->receive(m, timeout);
}


std::vector<hstd::Addr_t> FindRoute(hstd::Addr_t dest, hstd::Addr_t src)
{
	uint8_t Q[BOS::MAX_NUM_OF_MODULES] = { 0 };		// All nodes initially in Q (unvisited nodes)

	auto minUnvisitedUID = [](uint8_t *arr, uint8_t *Q) -> uint8_t {
		uint8_t index = 0, smallest = 0xFF; 

		if (!Q[0]) smallest = arr[0];	/* Consider first element as smallest */

		for (int i = 0; i < Service::numModules; i++) {
			if ((arr[i] < smallest) && !Q[i]) {
				smallest = arr[i];
				index = i;
			}
		}
		return index;
	};

	auto isQNotEmpty = [](uint8_t *Q) {
		char temp = 1;
		for (int i = 0; i < Service::numModules; i++)
			temp &= Q[i];
		return temp;
	};

	std::vector<hstd::Addr_t> route;
	hstd::Addr_t routePrev[BOS::MAX_NUM_OF_MODULES];
	uint8_t routeDist[BOS::MAX_NUM_OF_MODULES] = { 0 };

	uint8_t sourceID = src.getUID();
	uint8_t destID = dest.getUID();
	
	/* Initialization */
	// memset(Service::route, 0, sizeof(Service::route));
	memset(routeDist, 0xFF, sizeof(routeDist));
	// memset(routePrev, 0, sizeof(routePrev));

	routeDist[sourceID - 1] = 0;       				// Distance from source to source
	routePrev[sourceID - 1] = hstd::Addr_t();       // Previous node in optimal path initialization undefined
	
	/* Algorithm */
	uint8_t currentID = 0;
	while (!isQNotEmpty(Q)) {

		currentID = minUnvisitedUID(routeDist, Q) + 1;		// Source node in first case
		if (currentID == destID)
			goto finishedRoute;

		Q[currentID - 1] = 1;								// Remove u from Q 																					
		/* For each neighbor v where v is still in Q. */
		for (uint8_t p = 1; p <= 6; p++) {     				// Check all module ports
			if (!Service::hasValidInfoAt(currentID, p))		// There's a neighbor v at this port n
				continue;
			
			uint8_t nID = Service::getIDConnTo(currentID, p);
			if (Q[nID - 1])		// v is not in Q
				continue;												
				
			uint8_t newDist = routeDist[currentID - 1] + 1;		// Add one hop
			if (newDist < routeDist[nID - 1]) {     	// A shorter path to v has been found
				routeDist[nID - 1] = newDist; 
				routePrev[nID - 1] = hstd::Addr_t(currentID, p); 
			}
		}
	}	
		
finishedRoute:
		
	/* Build the virtual route */
	// currentID = routePrev[currentID - 1].getUID();
	route.push_back(hstd::Addr_t(Service::getAddrConnTo(routePrev[currentID - 1])));
	while (routePrev[currentID - 1].isValid()) {  		// Construct the shortest path with a stack route
		route.push_back(routePrev[currentID - 1]);		// Push the vertex onto the stack
		currentID = routePrev[currentID - 1].getUID(); 	// Traverse from target to source
	}

	std::reverse(route.begin(), route.end());
	return route;			
}


uint8_t FindRoute(uint8_t src, uint8_t dest)
{
	std::vector<hstd::Addr_t> route = FindRoute(hstd::Addr_t(dest, 1), hstd::Addr_t(src, 1));
	if (route.size() <= 0)
		return 0;
	return route.at(0).getPort();
}

uint8_t FindSourcePort(uint8_t srcID, uint8_t destID)
{
	std::vector<hstd::Addr_t> route = FindRoute(hstd::Addr_t(destID, 1), hstd::Addr_t(srcID, 1));
	if (route.size() <= 0)
		return 0;
	return route.at(0).getPort();
}


int ping(uint8_t destID)
{
	hstd::Message msg;
	msg.setSource(Service::myID);
	msg.setDest(destID);
	msg.setCode(CODE_ping);
	msg.setMessOnlyFlag(true); msg.setCLIOnlyFlag(true);

	Service::getInstance()->send(msg);
	Service::osDelay(300 * NumberOfHops(destID));

	if (!Service::getInstance()->receive(msg))
		return -1;
	return 0;
}


int assignIDToNeighbours(uint8_t id)
{
	hstd::Message msg;
	msg.setSource(0);
	msg.setDest(0);
	msg.setCode(CODE_module_id);
	msg.setTraceFlag(true); msg.setMessOnlyFlag(true);
	msg.getParams().append(uint8_t(0));	/* Change own ID */
	msg.getParams().append(id);
	msg.getParams().append(uint8_t(0));

	Service::getInstance()->send(msg);
	// No response to this message
	return 0;
}

int assignIDToAdjacent(uint8_t destID, uint8_t portNum, uint8_t newID)
{
	hstd::Message msg;
	msg.setSource(Service::myID);
	msg.setDest(destID);
	msg.setCode(CODE_module_id);
	msg.setTraceFlag(true); msg.setMessOnlyFlag(true);
	msg.getParams().append(uint8_t(1));	/* Change own ID */
	msg.getParams().append(newID);
	msg.getParams().append(portNum);

	Service::getInstance()->send(msg);
	// No response to this message
	return 0;
}

int sendHiMessage(uint8_t port, uint16_t partNumber)
{
	hstd::Message msg;
	msg.setSource(0);
	msg.setDest(0);
	msg.setCode(CODE_hi);
	msg.getParams().append(partNumber);
	msg.getParams().append(port);
	
	/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
	return Service::getInstance()->send(msg);
}

int ExploreNeighbors(uint8_t ignore)
{
	int result = 0; 

	/* Send Hi messages to adjacent neighbors */
	for (uint8_t port = 1; port <= NUM_OF_PORTS; port++) {
		if (port == ignore)
			continue;
		sendHiMessage(port, Service::partNumber);
		hstd::Message msg;
		Service::getInstance()->receive(msg);
		Service::neighbors[0][1] = msg.getParams().popui16();											/* Neighbor PN */
		Service::neighbors[0][0] = (uint16_t(msg.getSource().getUID()) << 8 ) + msg.getParams().popui8();		/* Neighbor ID + Neighbor own port */
		// Process Reply

	}
	
	return result;
}

int sendTopology(uint8_t destID, uint8_t maxID)
{
	hstd::Message msg;
	msg.setSource(Service::myID);
	msg.setDest(destID);
	msg.setCode(CODE_topology);
	msg.getParams().append((uint8_t *)(Service::array), (maxID * (BOS::MAX_NUM_OF_PORTS + 1) * 2));
	Service::getInstance()->send(msg);
	return 0;
}

int reverseAllButInPort(uint8_t destID)
{
	hstd::Message msg;
	msg.setSource(Service::myID);
	msg.setDest(destID);
	msg.setCode(CODE_port_dir);

	for (uint8_t p = 1 ; p <= BOS::MAX_NUM_OF_PORTS; p++)
		msg.getParams().append(uint8_t(BOS::PortDir::REVERSED));
	msg.getParams().append(uint8_t(BOS::PortDir::NORMAL)); /* Make sure the inport is not BOS::PortDir::REVERSED */

	Service::getInstance()->send(msg);
	return 0;
}

int SendMessageToModule(uint8_t destID, uint16_t code)
{
	hstd::Message msg;
	msg.setSource(Service::myID);
	msg.setDest(destID);
	msg.setCode(code);

	Service::getInstance()->send(msg);
	return 0;
}


int Explore(void)
{
	int result = 0;
	uint8_t PcPort = 0;
	
	uint8_t& myID = Service::myID; 		/* Master ID */

	uint8_t lastID = 0;
	uint8_t currentID = 1;
	
	/* >>> Step 1 - Reverse master ports and explore adjacent neighbors */
	ExploreNeighbors(PcPort);

	
	/* >>> Step 2 - Assign IDs to new modules & update the topology array */
	
	/* Step 2a - Assign IDs to new modules */
	for (uint8_t p = 1 ; p <= NUM_OF_PORTS; p++) {
		if (Service::neighbors[p - 1][0]) {
			assignIDToNeighbours(++currentID);
			Service::neighbors[p - 1][0] = (uint16_t(currentID) << 8) + uint8_t(Service::neighbors[p - 1][0]);
			Service::numModules = currentID;
			Service::osDelay(10);
		}
	}
	
	/* Step 2b - Update master topology array */
	Service::array[0][0] = Service::partNumber;					
	for (uint8_t p = 1 ; p <= NUM_OF_PORTS; p++) {
		if (!Service::neighbors[p - 1][0])
			continue;

		uint16_t temp = Service::neighbors[p - 1][0];
		uint8_t nID = uint8_t(temp >> 8);						/* Neighbor ID */
		uint8_t nPort = uint8_t(temp);							/* Neighbor port */

		/* Module 1 (master) */
		Service::array[0][p] = (nID << 3) | nPort;						/* Neighbor ID | Neighbor port */
		/* Rest of the neighbors */
		Service::array[nID - 1][0] = Service::neighbors[p - 1][1];		/* Neighbor PN */
		Service::array[nID - 1][nPort] = (myID << 3) | p;				/* Module 1 ID | Module 1 port */
	}		
	
	/* Step 2c - Ask neighbors to update their topology array */
	for (uint8_t i = 2 ; i <= currentID; i++) {
		sendTopology(i, currentID);
		Service::osDelay(60);
	}
	
	
	/* >>> Step 3 - Ask each new module to explore and repeat */
	
	while (lastID != currentID) {
		/* Update lastID */
		lastID = currentID;
		
		/* Scan all discovered modules */
		for (uint8_t i = 2 ; i <= currentID; i++) {
			/* Step 3a - Ask the module to reverse ports */
			reverseAllButInPort(i);
			Service::osDelay(10);
			
			/* Step 3b - Ask the module to explore adjacent neighbors */
			SendMessageToModule(i, CODE_explore_adj);
			Service::osDelay(100);		
		
			/* Step 3c - Assign IDs to new modules */
			for (uint8_t p = 1 ; p <= BOS::MAX_NUM_OF_PORTS; p++) {

				uint16_t temp = Service::neighbors2[p - 1][0];		/* Neighbor ID */
				uint8_t nID = uint8_t(temp >> 8);											
				if ((temp == 0) || (nID != 0))			/* IDed module */
					continue;

				assignIDToAdjacent(i, p, ++currentID);
				Service::numModules = currentID;			/* Update number of modules in the array */
				/* Modify neighbors table */
				Service::neighbors2[p - 1][0] = (uint16_t(currentID) << 8) + (uint8_t)(Service::neighbors2[p - 1][0]);
				Service::osDelay(10);
				
			}
			
			/* Step 3d - Update master topology array */
			for (uint8_t p = 1; p <= BOS::MAX_NUM_OF_PORTS; p++) {
				if (!Service::neighbors2[p - 1][0])
					continue;

				uint16_t temp = Service::neighbors2[p - 1][0];
				uint8_t n2ID = uint8_t(temp >> 8);					/* Neighbor ID */
				uint8_t n2Port = uint8_t(temp);						/* Neighbor port */	

				if (n2ID == 1)										/* Exclude Master */
					continue;			
				/* Update module i section */
				if (Service::array[i - 1][p] == 0)
					Service::array[i - 1][p] = (n2ID << 3 ) | n2Port;		/* Neighbor ID | Neighbor port */
				
				/* Update module i neighbors */
				if (Service::array[n2ID - 1][n2Port] == 0) {
					Service::array[n2ID - 1][0]	= Service::neighbors2[p - 1][1];	/* Neighbor PN */
					Service::array[n2ID - 1][n2Port] = ( i << 3 ) | p;				/* Module i ID | Module i port */								
				}
		}	
			
			/* Reset neighbors2 array */
			memset(Service::neighbors2, 0, sizeof(Service::neighbors2));
			
			/* Step 3e - Ask all discovered modules to update their topology array */
			for (uint8_t j = 2 ; j <= currentID; j++) {
				sendTopology(j, currentID);
				Service::osDelay(60);
			}
		}
	}

	
	/* >>> Step 4 - Make sure all connected modules have been discovered */
	
	ExploreNeighbors(PcPort);
	/* Check for any unIDed neighbors */
	for (uint8_t p = 1 ; p <= NUM_OF_PORTS; p++) {
		uint16_t temp = Service::neighbors[p-1][0];		/* Neighbor ID */
		uint8_t nID = uint8_t(temp >> 8);

		if ((temp != 0) && (nID == 0)) {		/* UnIDed module */
			result = -1;	// BOS_ERR_UnIDedModule
		}		
	}
	/* Ask other modules for any unIDed neighbors */
	for (uint8_t i = 2; i <= currentID; i++) {

		memset(Service::neighbors2, 0, sizeof(Service::neighbors2));

		SendMessageToModule(i, CODE_explore_adj);
		Service::osDelay(100);	

		/* Check for any unIDed neighbors */
		for (uint8_t p = 1; p <= BOS::MAX_NUM_OF_PORTS; p++) {
			uint16_t temp = Service::neighbors2[p - 1][0];		/* Neighbor ID */
			uint8_t n2ID = (uint8_t)(temp >> 8);											
			if (temp != 0 && n2ID == 0) {		/* UnIDed module */
				result = -1;	// BOS_ERR_UnIDedModule
			}
		}				
	}
	
	
	hstd::Message portMsg;
	/* >>> Step 5 - If no unIDed modules found, generate and distribute port directions */
	if (result)
		goto END;

	/* Step 5a - Virtually reset the state of master ports to BOS::PortDir::NORMAL */
	for (uint8_t p = 1; p <= NUM_OF_PORTS; p++)
		Service::arrayPortsDir[0] &= (~(0x8000 >> (p - 1)));		/* Set bit to zero */

	
	/* Step 5b - Update other modules ports starting from the last one */
	for (uint8_t i = currentID; i >= 2; i--) {
		for (uint8_t p = 1 ; p <= BOS::MAX_NUM_OF_PORTS; p++) {
			uint8_t nID = Service::getIDConnTo(i, p);				/* Neighbor ID */
			uint8_t nPort = Service::getPortConnTo(i, p);			/* Neighbor port */	

			if (!Service::hasValidInfoAt(i, p) or (Service::getPortDir(nID, nPort) == BOS::PortDir::REVERSED))	{
				/* If empty port leave BOS::PortDir::NORMAL */
				portMsg.getParams().append(uint8_t(BOS::PortDir::NORMAL));
				Service::setPortDir(i, p, BOS::PortDir::NORMAL);
			} else {
				portMsg.getParams().append(uint8_t(BOS::PortDir::REVERSED));
				Service::setPortDir(i, p, BOS::PortDir::REVERSED);			
			}
		}
		
		/* Step 5c - Check if an inport is BOS::PortDir::REVERSED */
		/* Find out the inport to this module from master */
		FindRoute(1, i);
		uint8_t justNextMod = Service::route[NumberOfHops(i)-1];				/* previous module = route[Number of hops - 1] */
		uint8_t port = FindRoute(i, justNextMod);
		/* Is the inport BOS::PortDir::REVERSED? */
		if ((justNextMod == i) || (portMsg.getParams()[port - 1] == uint8_t(BOS::PortDir::REVERSED)) )
			portMsg.getParams().append(uint8_t(BOS::PortDir::REVERSED));		/* Make sure the inport is BOS::PortDir::REVERSED */
		else
			portMsg.getParams().append(uint8_t(0));
		
		/* Step 5d - Update module ports directions */
		portMsg.setSource(Service::myID);
		portMsg.setDest(i);
		portMsg.setCode(CODE_port_dir);
		Service::getInstance()->send(portMsg);
		Service::osDelay(10);			
	}			

	/* Step 5e - Update master ports > all BOS::PortDir::NORMAL */
	
			
	/* >>> Step 6 - Test new port directions by pinging all modules */
	
	if (result)
		goto END; 

	Service::osDelay(100);
	// BOS.response = BOS_RESPONSE_MSG;		// Enable response for pings
	for (uint8_t i = 2; i <= Service::numModules; i++) {
		if (ping(i)) {
			result = -1;
			break;
		}
		// if (responseStatus == BOS_OK)
		// 	result = BOS_OK;
		// else if (responseStatus == BOS_ERR_NoResponse)
		// 	result = BOS_ERR_NoResponse;
	}
	
	/* >>> Step 7 - Save all (topology and port directions) in RO/EEPROM */
	if (result)
		goto END;
	/* Save data in the master */
	// SaveToRO();
	// SaveEEportsDir();
	Service::osDelay(100);
	/* Ask other modules to save their data too */
	SendMessageToModule(hstd::Addr_t::BROADCAST_UID, CODE_exp_eeprom);
	

END:
	return result;
}
