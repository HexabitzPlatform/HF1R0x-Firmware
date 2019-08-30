#ifndef MODULE_H
#define MODULE_H

#include "hal/Serial.h"
#include "hexabitz/BOSMessage.h"
#include "hexabitz/BOSFrame.h"

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include <memory>
#include <list>

#define NUM_OF_PORTS 					1
#define MAX_NUM_OF_PORTS				6
#define	MAX_NUM_OF_MODULES				25

#define BOS_MULTICAST					254
#define BOS_BROADCAST 					255

#define	NumberOfHops(i)					(Service::routeDist[i - 1])

enum UartDirection_e {
	NORMAL, 
	REVERSED
};


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
	static uint8_t getPortDir(uint8_t id, uint8_t port)					{ return arrayPortsDir[id - 1] & (0x8000 >> (port - 1));}
	static void setPortDir(uint8_t id, uint8_t port, uint8_t dir)
	{
		if (dir == REVERSED)
			arrayPortsDir[id - 1] |= (0x8000 >> (port - 1));
		else
			arrayPortsDir[id - 1] &= (~(0x8000 >> (port - 1)));	

	}
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

	static uint16_t neighbors[NUM_OF_PORTS][2];
	static uint16_t neighbors2[MAX_NUM_OF_PORTS][2];
	static uint16_t array[MAX_NUM_OF_MODULES][MAX_NUM_OF_PORTS + 1];
	static uint16_t arrayPortsDir[MAX_NUM_OF_MODULES];

	static uint8_t route[MAX_NUM_OF_MODULES];
	static uint8_t routeDist[MAX_NUM_OF_MODULES];
};



/*************************************************************/

class ProxyModule {

public:
	long getID(void) const;
	long setID(long newID);

	bool isMaster(void) const { return (id_ == 1); }
	bool isValidID(void) const { return (id_ >= 1); }

	std::string getPartStr(void) const { return info_; }

public:
	virtual bool send(const hstd::Message& m);
	virtual bool receive(hstd::Message& m, long timeout = -1);

public:
	ProxyModule(std::string partStr);
	~ProxyModule(void);

protected:
	long id_;
	const std::string info_;
};

/*************************************************************/


class H09R0: public ProxyModule {


public:
	float getTemp(void)
	{
		hstd::Message m;
		m.setSource(1);
		m.setDest(1);
		m.setCode(1);

		return 0;
	}

public:
	H09R0(std::string partName): ProxyModule(partName)
	{

	}

	~H09R0(void)
	{

	}
};

/*************************************************************/

class H01R0: public ProxyModule {

public:

	void setRGB(int red, int green, int blue)
	{

	}

public:
	H01R0(std::string partName): ProxyModule(partName)
	{

	}

	~H01R0(void)
	{

	}
};

/*************************************************************/



#endif // MODULE_H