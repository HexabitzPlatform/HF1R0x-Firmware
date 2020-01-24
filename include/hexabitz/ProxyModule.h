#ifndef PROXYMODULE_H
#define PROXYMODULE_H

#include "hexabitz/Service.h"
#include "hexabitz/BOS.h"
#include "hexabitz/BOSFrame.h"
#include "hexabitz/BOSMessage.h"

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include <memory>
#include <list>


/*************************************************************/

class ProxyModule {
public:
	hstd::uid_t getUID(void) const;
	void setUID(hstd::uid_t newID);

	int getNumOfPorts(void) const;

	bool isMaster(void) const;

protected:
	bool isValidID(void) const;
	void setNumOfPorts(int ports);

public:
	std::string getPartStr(void) const;
	BOS::module_pn_e getPartNum(void) const;
	uint16_t getPartNum_ui16(void) const;

public:
	virtual bool send(hstd::Message m);
	virtual bool receive(hstd::Message& m, long timeout = -1);

public:
	ProxyModule(std::string part, int numPorts, hstd::uid_t uid);
	ProxyModule(std::string part, int numPorts = -1);
	ProxyModule(BOS::module_pn_e part);

	~ProxyModule(void);

protected:
	hstd::uid_t id_;
	int numOfPorts_ = 0;
	std::string info_;
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
	H09R0(void): ProxyModule(BOS::H09R0)
	{

	}

	~H09R0(void)
	{

	}
};

/*************************************************************/

class H01R0: public ProxyModule {

public:
	bool setRGB(int red, int green, int blue, int intensity);


public:
	H01R0(void): ProxyModule(BOS::H01R0)
	{
		id_ = Service::getInstance()->getIDOfPartNum(BOS::H01R0, 1);
		std::cout << "ID to BOS::H01R0: " << id_ << std::endl;
	}

	~H01R0(void)
	{

	}
};

/*************************************************************/



#endif /* PROXYMODULE_H */