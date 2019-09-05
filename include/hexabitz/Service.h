#ifndef SERVICE_H
#define SERVICE_H

#include "helper/helper.h"

#include "hal/Serial.h"
#include "hexabitz/BOSFrame.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/BOSMessageBuilder.h"

#include <memory>


#define	NumberOfHops(i)					(Service::routeDist[i - 1])

class ProxyModule;

class Service {

public:
	static Service *getInstance(void);
	static void osDelay(int milliseconds);

public:
	bool init(const char *pathname);
	int setProxy(ProxyModule *module);

public:
	static bool hasValidInfoAt(uint8_t id, uint8_t port)				{ return array[id - 1][port] != 0; }
	static uint8_t getIDConnTo(uint8_t id, uint8_t port)				{ return (array[id - 1][port] >> 3); }
	static uint8_t getPortConnTo(uint8_t id, uint8_t port)				{ return (array[id - 1][port] & 0x0007); }

	static BOS::PortDir getPortDir(uint8_t id, uint8_t port);
	static void setPortDir(uint8_t id, uint8_t port, BOS::PortDir dir);

	static hstd::Addr_t getAddrConnTo(hstd::Addr_t addr)
	{
		return hstd::Addr_t(getIDConnTo(addr.getUID(), addr.getPort()));
	}


public:
	virtual bool send(const hstd::Message& m);
	virtual bool receive(hstd::Message& m, long timeout = -1);

private:
	virtual bool send(const hstd::Frame& f);
	virtual bool receive(hstd::Frame& f, long timeout = -1);

private:
	Service(void): serial_(nullptr)	{ }
	~Service(void)  				{ serial_.end(); }

private:
	// Save Routing Table!

private:
	HardwareSerial serial_;
	std::shared_ptr<ProxyModule> module_;

public:
	static Service *self_;
	static enum BOS::module_pn_e partNumber;
	static uint8_t myID;
	static uint8_t numModules;

	static uint16_t neighbors[BOS::MAX_NUM_OF_PORTS][2];
	static uint16_t neighbors2[BOS::MAX_NUM_OF_PORTS][2];
	static uint16_t array[BOS::MAX_NUM_OF_MODULES][BOS::MAX_NUM_OF_PORTS + 1];
	static uint16_t arrayPortsDir[BOS::MAX_NUM_OF_MODULES];

	static uint8_t route[BOS::MAX_NUM_OF_MODULES];
	static uint8_t routeDist[BOS::MAX_NUM_OF_MODULES];
};


#endif /* SERVICE_H */