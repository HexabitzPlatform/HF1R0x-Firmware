#ifndef SERVICE_H
#define SERVICE_H

#include "helper/helper.h"

#include "hal/Serial.h"
#include "hexabitz/BOSFrame.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/ModuleInfo.h"
#include "hexabitz/BOSMessageBuilder.h"

#include <errno.h>

#include <memory>
#include <vector>



class ProxyModule;

class Service {

public:
	static Service *getInstance(void);

public:
	static void osDelay(int milliseconds);

public:
	int init(const char *pathname);

	void setOwn(std::shared_ptr<ProxyModule> module);
	std::shared_ptr<ProxyModule> getOwn(void);

private:
	int ping(uint8_t destID);
	int assignIDToNeigh(hstd::uid_t id, hstd::port_t portNum);
	int assignIDToAdjacent(uint8_t destID, uint8_t portNum, uint8_t newID);
	int sayHiToNeighbour(uint8_t portOut, enum BOS::module_pn_e& part, hstd::Addr_t& neigh);
	NeighboursInfo ExploreNeighbors(uint8_t ignore);
	int ExploreAdjacentOf(hstd::Addr_t addr, NeighboursInfo& info);
	void changePortDir(int port, enum BOS::PortDir dir);
	int syncTopologyTo(hstd::uid_t destID);
	int synPortDir(hstd::uid_t dest);
	int reverseAllButInPort(hstd::uid_t destID);
	int broadcastToSave(void);
	int Explore(void);

private:
	std::vector<hstd::Addr_t> FindRoute(hstd::Addr_t dest);
	std::vector<hstd::Addr_t> FindRoute(hstd::uid_t destID);
	hstd::port_t FindSrcPortFor(hstd::uid_t destID);

public:
	int send(const hstd::Message& m);
	int receive(hstd::Message& m, long timeout = -1);

private:
	int send(const hstd::Frame& f);
	int receive(hstd::Frame& f, long timeout = -1);

private:
	Service(void);
	~Service(void);

private:
	HardwareSerial serial_;

private:
	static Service *self_;
	static const int DEFAULT_BAUDRATE = 921600;

private:
	std::shared_ptr<ProxyModule> owner_;
	uint8_t num_modules_;
	ModulesInfo info_;
};


#endif /* SERVICE_H */