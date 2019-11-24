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
	int init(const std::string& pathname);

	void setOwn(std::shared_ptr<ProxyModule> module);
	std::shared_ptr<ProxyModule> getOwn(void);

	const ModulesInfo& getModulesInfo(void) const 	{ return info_; }

private:
	int assignIDToNeigh(hstd::uid_t id, hstd::port_t portNum);
	int assignIDToAdjacent(hstd::uid_t destID, hstd::port_t port, hstd::uid_t newID);

	int sayHiToNeighbour(hstd::port_t port, enum BOS::module_pn_e& part, hstd::Addr_t& neigh);
	int ExploreNeighbors(hstd::port_t ignore, NeighboursInfo& info);
	int ExploreAdjacentOf(hstd::Addr_t addr, NeighboursInfo& info);

	void changePortDir(hstd::port_t port, enum BOS::PortDir dir);
	int syncTopologyTo(hstd::uid_t destID);
	int synPortDir(hstd::uid_t dest);
	int reverseAllButInPort(hstd::uid_t destID);
	int broadcastToSave(void);

public:
	int ping(hstd::uid_t destID);
	int ping(hstd::uid_t destID, hstd::uid_t srcID);
	int Explore(void);

private:
	std::vector<hstd::Addr_t> FindRoute(hstd::Addr_t dest);
	std::vector<hstd::Addr_t> FindRoute(hstd::uid_t destID);
	hstd::port_t FindSrcPortFor(hstd::uid_t destID);

private:
	static void enterCriticalSection(void);
	static void exitCriticalSection(void);

public:
	int send(hstd::Message m);
	int receive(hstd::Message& m, long timeout = -1);

	hstd::uid_t getIDOfPartNum(enum BOS::module_pn_e partNum, unsigned num) const { return info_.getUIDOf(partNum, num); }

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
	inline static const std::string DEF_CACHE_FILENAME = "cache.bin";

private:
	std::shared_ptr<ProxyModule> owner_;
	int num_modules_;
	ModulesInfo info_;
};


#endif /* SERVICE_H */