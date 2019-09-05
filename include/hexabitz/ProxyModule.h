#ifndef PROXYMODULE_H
#define PROXYMODULE_H

#include "hal/Serial.h"
#include "hexabitz/BOS.h"
#include "hexabitz/BOSFrame.h"
#include "hexabitz/BOSMessage.h"

#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include <memory>
#include <list>

#define NUM_OF_PORTS 					1


enum UartDirection_e {
	NORMAL, 
	REVERSED
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



#endif /* PROXYMODULE_H */