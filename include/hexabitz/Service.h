#ifndef SERVICE_H
#define SERVICE_H

#include "helper/helper.h"

#include "hal/Serial.h"
#include "hexabitz/BOSFrame.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/BOSMessageBuilder.h"

#include <memory>
#include <vector>


#define	NumberOfHops(i)					(Service::routeDist[i - 1])

class ProxyModule;

class Service {

public:
	static Service *getInstance(void);

public:
	static void osDelay(int milliseconds);

public:
	bool init(const char *pathname);

	void setOwn(std::shared_ptr<ProxyModule> module);
	std::shared_ptr<ProxyModule> getOwn(void);

public:
	bool hasValidInfoAt(uint8_t id, uint8_t port);

	// void setModuleConnAt(hstd::Addr_t module, hstd::Addr_t connectedAt);
	uint8_t getIDConnTo(uint8_t id, uint8_t port);			
	uint8_t getPortConnTo(uint8_t id, uint8_t port);				
	hstd::Addr_t getAddrConnTo(hstd::Addr_t addr);
	
	void setPortDir(uint8_t id, uint8_t port, BOS::PortDir dir);
	BOS::PortDir getPortDir(uint8_t id, uint8_t port);


private:
	std::vector<hstd::Addr_t> FindRoute(hstd::Addr_t dest, hstd::Addr_t src);
	uint8_t FindRoute(uint8_t src, uint8_t dest);
	uint8_t FindSourcePort(uint8_t srcID, uint8_t destID);

public:
	virtual bool send(const hstd::Message& m);
	virtual bool receive(hstd::Message& m, long timeout = -1);

private:
	virtual bool send(const hstd::Frame& f);
	virtual bool receive(hstd::Frame& f, long timeout = -1);

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

	uint16_t neighbors_[BOS::MAX_NUM_OF_PORTS][2];
	uint16_t neighbors2_[BOS::MAX_NUM_OF_PORTS][2];
	uint16_t modulesInfo_[BOS::MAX_NUM_OF_MODULES][BOS::MAX_NUM_OF_PORTS + 1];
	uint16_t modulesPortsDirInfo_[BOS::MAX_NUM_OF_MODULES];
};


#endif /* SERVICE_H */