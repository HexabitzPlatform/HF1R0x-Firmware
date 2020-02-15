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

#endif /* PROXYMODULE_H */