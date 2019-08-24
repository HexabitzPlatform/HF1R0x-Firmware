#include "hexabitz/Module.h"
#include "helper/helper.h"

#include "hal/Serial.h"
#include "hexabitz/BOSMessage.h"

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

Service *Service::self_ = nullptr;
uint8_t Service::myID = 0;

enum modulePartNumbers_e Service::partNumber = _H01R0;

uint8_t Service::numModules = 50;
uint16_t Service::neighbors[NUM_OF_PORTS][2] = { { 0 } };
uint16_t Service::neighbors2[MAX_NUM_OF_PORTS][2] = { { 0 } };
uint16_t Service::array[MAX_NUM_OF_MODULES][MAX_NUM_OF_PORTS + 1] = { { 0 } };
uint16_t Service::arrayPortsDir[MAX_NUM_OF_MODULES] = { 0 };
uint8_t Service::route[MAX_NUM_OF_MODULES] = { 0 };
uint8_t Service::routeDist[MAX_NUM_OF_MODULES] = { 0 };

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

bool Service::send(const hstd::Message& msg)
{
	std::vector<hstd::Frame> list = hstd::buildFramesFromMessage(msg);
	for (auto& f: list) {
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

		f.param.reset();
		buffer.append(uint8_t(serial_.read()));
		if (f.fromBuffer(buffer))
			return true;
	}
	return false;
}


ProxyModule::ProxyModule(std::string partStr): id_(-1), info_(partStr)
{

}

ProxyModule::~ProxyModule(void)
{

}

long ProxyModule::getID(void) const
{
	return id_;
}

long ProxyModule::setID(long newID)
{
	long oldID = id_;
	id_ = newID;
	return oldID;
}

bool ProxyModule::send(const hstd::Message& m)
{
	return false;
}

bool ProxyModule::receive(hstd::Message& m, long timeout)
{
	return false;
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

// int ping(uint8_t destID)
// {
// 	hstd::Message msg;
// 	msg.setSource(Service::myID);
// 	msg.setDest(destID);
// 	msg.setCode(CODE_ping);
// 	msg.setMessOnlyFlag(true); msg.setCLIOnlyFlag(true);

// 	Service::getInstance()->send(msg);
// 	Service::osDelay(300 * NumberOfHops(destID));

// 	if (!Service::getInstance()->receive(msg))
// 		return -1;
// 	return 0;
// }


// int assignIDToNeighbours(uint8_t id)
// {
// 	hstd::Message msg;
// 	msg.setSource(0);
// 	msg.setDest(0);
// 	msg.setCode(CODE_module_id);
// 	msg.setTraceFlag(true); msg.setMessOnlyFlag(true);
// 	msg.getParams().append(uint8_t(0));	/* Change own ID */
// 	msg.getParams().append(id);
// 	msg.getParams().append(uint8_t(0));

// 	Service::getInstance()->send(msg);
// 	// No response to this message
// 	return 0;
// }

// int assignIDToAdjacent(uint8_t destID, uint8_t portNum, uint8_t newID)
// {
// 	hstd::Message msg;
// 	msg.setSource(Service::myID);
// 	msg.setDest(destID);
// 	msg.setCode(CODE_module_id);
// 	msg.setTraceFlag(true); msg.setMessOnlyFlag(true);
// 	msg.getParams().append(uint8_t(1));	/* Change own ID */
// 	msg.getParams().append(newID);
// 	msg.getParams().append(portNum);

// 	Service::getInstance()->send(msg);
// 	// No response to this message
// 	return 0;
// }

// int sendHiMessage(uint8_t port, uint16_t partNumber)
// {
// 	hstd::Message msg;
// 	msg.setSource(0);
// 	msg.setDest(0);
// 	msg.setCode(CODE_hi);
// 	msg.getParams().append(partNumber);
// 	msg.getParams().append(port);
	
// 	/* Port, Source = 0 (myID), Destination = 0 (adjacent neighbor), message code, number of parameters */
// 	return Service::getInstance()->send(msg);
// }

// int ExploreNeighbors(uint8_t ignore)
// {
// 	int result = 0; 

// 	/* Send Hi messages to adjacent neighbors */
// 	for (uint8_t port = 1; port <= NUM_OF_PORTS; port++) {
// 		if (port == ignore)
// 			continue;
// 		sendHiMessage(port, Service::partNumber);
// 		hstd::Message msg;
// 		Service::getInstance()->receive(msg);
// 		Service::neighbors[0][1] = msg.getParams().popui16();											/* Neighbor PN */
// 		Service::neighbors[0][0] = (uint16_t(msg.getSource()) << 8 ) + msg.getParams().popui8();		/* Neighbor ID + Neighbor own port */
// 		// Process Reply

// 	}
	
// 	return result;
// }

// int sendTopology(uint8_t destID, uint8_t maxID)
// {
// 	hstd::Message msg;
// 	msg.setSource(Service::myID);
// 	msg.setDest(destID);
// 	msg.setCode(CODE_topology);
// 	msg.getParams().append((uint8_t *)(Service::array), (maxID * (MAX_NUM_OF_PORTS + 1) * 2));
// 	Service::getInstance()->send(msg);
// 	return 0;
// }

// int reverseAllButInPort(uint8_t destID)
// {
// 	hstd::Message msg;
// 	msg.setSource(Service::myID);
// 	msg.setDest(destID);
// 	msg.setCode(CODE_port_dir);

// 	for (uint8_t p = 1 ; p <= MAX_NUM_OF_PORTS; p++)
// 		msg.getParams().append(uint8_t(REVERSED));
// 	msg.getParams().append(uint8_t(NORMAL)); /* Make sure the inport is not reversed */

// 	Service::getInstance()->send(msg);
// 	return 0;
// }

// int SendMessageToModule(uint8_t destID, uint16_t code)
// {
// 	hstd::Message msg;
// 	msg.setSource(Service::myID);
// 	msg.setDest(destID);
// 	msg.setCode(code);

// 	Service::getInstance()->send(msg);
// 	return 0;
// }

// uint8_t QnotEmpty(uint8_t *Q)
// {		
// 	char temp = 1;
// 	for (int i = 0; i < Service::numModules; i++)
// 		temp &= Q[i];
	
// 	return temp;
// }


// uint8_t minArr(uint8_t *arr, uint8_t *Q)
// {
// 	uint8_t index = 0;
// 	uint8_t smallest = 0xFF; 

// 	/* Consider first element as smallest */
// 	if (!Q[0])						// Not visited yet
// 		smallest = arr[0];

// 	for (int i = 0; i < Service::numModules; i++) {
// 		if ((arr[i] < smallest) && !Q[i]) {
// 			smallest = arr[i];
// 			index = i;
// 		}
// 	}
	
// 	return index;
// }

// uint8_t FindRoute(uint8_t sourceID, uint8_t desID)
// {
// 	uint8_t Q[50] = {0};		// All nodes initially in Q (unvisited nodes)
// 	uint8_t routePrev[MAX_NUM_OF_MODULES] = { 0 };
	
// 	/* Initialization */
// 	memset(Service::route, 0, sizeof(Service::route));
// 	memset(Service::routeDist, 0xFF, sizeof(Service::routeDist));
// 	memset(routePrev, 0, sizeof(routePrev));

// 	Service::routeDist[sourceID - 1] = 0;       // Distance from source to source
// 	routePrev[sourceID - 1] = 0;      	// Previous node in optimal path initialization undefined
	
// 	/* Algorithm */
// 	uint8_t currentID = 0;
// 	while (!QnotEmpty(Q)) {

// 		currentID = minArr(Service::routeDist, Q) + 1;						// Source node in first case
// 		if (currentID == desID)
// 			goto finishedRoute;

// 		Q[currentID - 1] = 1;													// Remove u from Q 																					
// 		/* For each neighbor v where v is still in Q. */
// 		for (uint8_t p = 1; p <= 6; p++) {     		// Check all module ports
// 			if (!Service::hasValidInfoAt(currentID, p))					// There's a neighbor v at this port n
// 				continue;
			
// 			uint8_t nID = Service::getIDConnTo(currentID, p);
// 			if (Q[nID - 1])		// v is still in Q
// 				continue;												
				
// 			uint8_t newDist = Service::routeDist[currentID - 1] + 1;					// Add one hop
// 			if (newDist < Service::routeDist[nID - 1]) {     		// A shorter path to v has been found
// 				Service::routeDist[nID - 1] = newDist; 
// 				routePrev[nID - 1] = currentID; 
// 			}
// 		}
// 	}	
		
// finishedRoute:
		
// 	/* Build the virtual route */
// 	uint8_t j = 0;
// 	while (routePrev[currentID - 1]) {  		// Construct the shortest path with a stack route
// 		Service::route[j++] = currentID;					// Push the vertex onto the stack
// 		currentID = routePrev[currentID - 1];  	// Traverse from target to source
// 	}
	
// 	/* Check which port leads to the correct module */
// 	for(uint8_t p = 1; p <= 6; p++)	{					
// 		if (Service::hasValidInfoAt(sourceID, p) and (Service::getIDConnTo(sourceID, p) == Service::route[Service::routeDist[desID - 1] - 1]))
// 			return p;
// 	}	

// 	return 0;			
// }

// int Explore(void)
// {
// 	int result = 0;
// 	uint8_t PcPort = 0;
	
// 	uint8_t& myID = Service::myID; 		/* Master ID */

// 	uint8_t lastID = 0;
// 	uint8_t currentID = 1;
	
// 	/* >>> Step 1 - Reverse master ports and explore adjacent neighbors */
// 	ExploreNeighbors(PcPort);

	
// 	/* >>> Step 2 - Assign IDs to new modules & update the topology array */
	
// 	/* Step 2a - Assign IDs to new modules */
// 	for (uint8_t p = 1 ; p <= NUM_OF_PORTS; p++) {
// 		if (Service::neighbors[p - 1][0]) {
// 			assignIDToNeighbours(++currentID);
// 			Service::neighbors[p - 1][0] = (uint16_t(currentID) << 8) + uint8_t(Service::neighbors[p - 1][0]);
// 			Service::numModules = currentID;
// 			Service::osDelay(10);
// 		}
// 	}
	
// 	/* Step 2b - Update master topology array */
// 	Service::array[0][0] = Service::partNumber;					
// 	for (uint8_t p = 1 ; p <= NUM_OF_PORTS; p++) {
// 		if (!Service::neighbors[p - 1][0])
// 			continue;

// 		uint16_t temp = Service::neighbors[p - 1][0];
// 		uint8_t nID = uint8_t(temp >> 8);						/* Neighbor ID */
// 		uint8_t nPort = uint8_t(temp);							/* Neighbor port */

// 		/* Module 1 (master) */
// 		Service::array[0][p] = (nID << 3) | nPort;						/* Neighbor ID | Neighbor port */
// 		/* Rest of the neighbors */
// 		Service::array[nID - 1][0] = Service::neighbors[p - 1][1];		/* Neighbor PN */
// 		Service::array[nID - 1][nPort] = (myID << 3) | p;				/* Module 1 ID | Module 1 port */
// 	}		
	
// 	/* Step 2c - Ask neighbors to update their topology array */
// 	for (uint8_t i = 2 ; i <= currentID; i++) {
// 		sendTopology(i, currentID);
// 		Service::osDelay(60);
// 	}
	
	
// 	/* >>> Step 3 - Ask each new module to explore and repeat */
	
// 	while (lastID != currentID) {
// 		/* Update lastID */
// 		lastID = currentID;
		
// 		/* Scan all discovered modules */
// 		for (uint8_t i = 2 ; i <= currentID; i++) {
// 			/* Step 3a - Ask the module to reverse ports */
// 			reverseAllButInPort(i);
// 			Service::osDelay(10);
			
// 			/* Step 3b - Ask the module to explore adjacent neighbors */
// 			SendMessageToModule(i, CODE_explore_adj);
// 			Service::osDelay(100);		
		
// 			/* Step 3c - Assign IDs to new modules */
// 			for (uint8_t p = 1 ; p <= MAX_NUM_OF_PORTS; p++) {

// 				uint16_t temp = Service::neighbors2[p - 1][0];		/* Neighbor ID */
// 				uint8_t nID = uint8_t(temp >> 8);											
// 				if ((temp == 0) || (nID != 0))			/* IDed module */
// 					continue;

// 				assignIDToAdjacent(i, p, ++currentID);
// 				Service::numModules = currentID;			/* Update number of modules in the array */
// 				/* Modify neighbors table */
// 				Service::neighbors2[p - 1][0] = (uint16_t(currentID) << 8) + (uint8_t)(Service::neighbors2[p - 1][0]);
// 				Service::osDelay(10);
				
// 			}
			
// 			/* Step 3d - Update master topology array */
// 			for (uint8_t p = 1; p <= MAX_NUM_OF_PORTS; p++) {
// 				if (!Service::neighbors2[p - 1][0])
// 					continue;

// 				uint16_t temp = Service::neighbors2[p - 1][0];
// 				uint8_t n2ID = uint8_t(temp >> 8);					/* Neighbor ID */
// 				uint8_t n2Port = uint8_t(temp);						/* Neighbor port */	

// 				if (n2ID == 1)										/* Exclude Master */
// 					continue;			
// 				/* Update module i section */
// 				if (Service::array[i - 1][p] == 0)
// 					Service::array[i - 1][p] = (n2ID << 3 ) | n2Port;		/* Neighbor ID | Neighbor port */
				
// 				/* Update module i neighbors */
// 				if (Service::array[n2ID - 1][n2Port] == 0) {
// 					Service::array[n2ID - 1][0]	= Service::neighbors2[p - 1][1];	/* Neighbor PN */
// 					Service::array[n2ID - 1][n2Port] = ( i << 3 ) | p;				/* Module i ID | Module i port */								
// 				}
// 		}	
			
// 			/* Reset neighbors2 array */
// 			memset(Service::neighbors2, 0, sizeof(Service::neighbors2));
			
// 			/* Step 3e - Ask all discovered modules to update their topology array */
// 			for (uint8_t j = 2 ; j <= currentID; j++) {
// 				sendTopology(j, currentID);
// 				Service::osDelay(60);
// 			}
// 		}
// 	}

	
// 	/* >>> Step 4 - Make sure all connected modules have been discovered */
	
// 	ExploreNeighbors(PcPort);
// 	/* Check for any unIDed neighbors */
// 	for (uint8_t p = 1 ; p <= NUM_OF_PORTS; p++) {
// 		uint16_t temp = Service::neighbors[p-1][0];		/* Neighbor ID */
// 		uint8_t nID = uint8_t(temp >> 8);

// 		if ((temp != 0) && (nID == 0)) {		/* UnIDed module */
// 			result = -1;	// BOS_ERR_UnIDedModule
// 		}		
// 	}
// 	/* Ask other modules for any unIDed neighbors */
// 	for (uint8_t i = 2; i <= currentID; i++) {

// 		memset(Service::neighbors2, 0, sizeof(Service::neighbors2));

// 		SendMessageToModule(i, CODE_explore_adj);
// 		Service::osDelay(100);	

// 		/* Check for any unIDed neighbors */
// 		for (uint8_t p = 1; p <= MAX_NUM_OF_PORTS; p++) {
// 			uint16_t temp = Service::neighbors2[p - 1][0];		/* Neighbor ID */
// 			uint8_t n2ID = (uint8_t)(temp >> 8);											
// 			if (temp != 0 && n2ID == 0) {		/* UnIDed module */
// 				result = -1;	// BOS_ERR_UnIDedModule
// 			}
// 		}				
// 	}
	
	
// 	hstd::Message portMsg;
// 	/* >>> Step 5 - If no unIDed modules found, generate and distribute port directions */
// 	if (result)
// 		goto END;

// 	/* Step 5a - Virtually reset the state of master ports to Normal */
// 	for (uint8_t p = 1; p <= NUM_OF_PORTS; p++)
// 		Service::arrayPortsDir[0] &= (~(0x8000 >> (p - 1)));		/* Set bit to zero */

	
// 	/* Step 5b - Update other modules ports starting from the last one */
// 	for (uint8_t i = currentID; i >= 2; i--) {
// 		for (uint8_t p = 1 ; p <= MAX_NUM_OF_PORTS; p++) {
// 			uint8_t nID = Service::getIDConnTo(i, p);				/* Neighbor ID */
// 			uint8_t nPort = Service::getPortConnTo(i, p);			/* Neighbor port */	

// 			if (!Service::hasValidInfoAt(i, p) or (Service::getPortDir(nID, nPort) == REVERSED))	{
// 				/* If empty port leave normal */
// 				portMsg.getParams().append(uint8_t(NORMAL));
// 				Service::setPortDir(i, p, NORMAL);
// 			} else {
// 				portMsg.getParams().append(uint8_t(REVERSED));
// 				Service::setPortDir(i, p, REVERSED);			
// 			}
// 		}
		
// 		/* Step 5c - Check if an inport is reversed */
// 		/* Find out the inport to this module from master */
// 		FindRoute(1, i);
// 		uint8_t justNextMod = Service::route[NumberOfHops(i)-1];				/* previous module = route[Number of hops - 1] */
// 		uint8_t port = FindRoute(i, justNextMod);
// 		/* Is the inport reversed? */
// 		if ((justNextMod == i) || (portMsg.getParams()[port - 1] == REVERSED) )
// 			portMsg.getParams().append(uint8_t(REVERSED));		/* Make sure the inport is reversed */
// 		else
// 			portMsg.getParams().append(uint8_t(0));
		
// 		/* Step 5d - Update module ports directions */
// 		portMsg.setSource(Service::myID);
// 		portMsg.setDest(i);
// 		portMsg.setCode(CODE_port_dir);
// 		Service::getInstance()->send(portMsg);
// 		Service::osDelay(10);			
// 	}			

// 	/* Step 5e - Update master ports > all normal */
	
			
// 	/* >>> Step 6 - Test new port directions by pinging all modules */
	
// 	if (result)
// 		goto END; 

// 	Service::osDelay(100);
// 	// BOS.response = BOS_RESPONSE_MSG;		// Enable response for pings
// 	for (uint8_t i = 2; i <= Service::numModules; i++) {
// 		if (ping(i)) {
// 			result = -1;
// 			break;
// 		}
// 		// if (responseStatus == BOS_OK)
// 		// 	result = BOS_OK;
// 		// else if (responseStatus == BOS_ERR_NoResponse)
// 		// 	result = BOS_ERR_NoResponse;
// 	}
	
// 	/* >>> Step 7 - Save all (topology and port directions) in RO/EEPROM */
// 	if (result)
// 		goto END;
// 	/* Save data in the master */
// 	// SaveToRO();
// 	// SaveEEportsDir();
// 	Service::osDelay(100);
// 	/* Ask other modules to save their data too */
// 	SendMessageToModule(BOS_BROADCAST, CODE_exp_eeprom);
	

// END:
// 	return result;
// }


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
	while (1) {
		hstd::Message m;
		m.setSource(uint8_t(0));
		m.setDest(uint8_t(1));
		m.setCode(CODE_hi);
		m.setMessOnlyFlag(true);
		m.setCLIOnlyFlag(true);
		m.setTraceFlag(true);

		// std::cout << "Sending..." << std::endl;
		// std::cout << m << std::endl;
		// std::cout << std::string(m) << std::endl;
		Service::getInstance()->send(m);
		if (Service::getInstance()->receive(m)) {
			std::cout << "Received message " << m << std::endl;
		}

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

	Service::getInstance()->init("/dev/ttyUSB0");

	testBinaryMessage();

	std::cout << "Closing Program" << std::endl;
	return 0;
}

#endif