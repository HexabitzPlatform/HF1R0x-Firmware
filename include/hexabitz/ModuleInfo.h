#ifndef MODULE_INFO_H
#define MODULE_INFO_H
	
#include "helper/helper.h"
#include "hexabitz/BOS.h"
#include "hexabitz/BOSMessage.h"

#include <errno.h>

class ProxyModule;

/***************************************** Neighbours Info Class *****************************************/

class NeighboursInfo {

public:
	BinaryBuffer toBinaryBuffer(void) const;
	void fromBinaryBuffer(BinaryBuffer buffer);

	void reset(void);

public:
	bool hasInfo(hstd::port_t port) const;
	bool hasIDedInfo(hstd::port_t port) const;
	bool hasUnIDedInfo(hstd::port_t port) const;
	bool hasAllIDedInfo(void) const;

	void setAddrInfoFor(hstd::port_t port, hstd::Addr_t addr);
	void setUIDInfoFor(hstd::port_t port, hstd::uid_t uid);
	void setPortInfoFor(hstd::port_t port, hstd::port_t neighPort);

	void setPartInfoFor(hstd::port_t port, enum BOS::module_pn_e part);

public:
	enum BOS::module_pn_e getPartEnumAt(hstd::port_t port) const;

	hstd::Addr_t getAddrAt(hstd::port_t port) const;
	hstd::uid_t getUIDAt(hstd::port_t port) const;
	hstd::port_t getPortAt(hstd::port_t port) const;

public:
	NeighboursInfo(void);
	~NeighboursInfo(void);

private:
	uint16_t array_[BOS::MAX_NUM_OF_PORTS][2];

private:
	static const int ADDR_NDX = 0;
	static const int PART_NDX = 1;
};

/*********************************************************************************************************/


/******************************************* Module Info Class *******************************************/

class ModulesInfo {

public:
	BinaryBuffer toBinaryBuffer(int num = -1) const;
	void fromBinaryBuffer(BinaryBuffer buffer);

	void reset(void);

public:
	bool hasConnInfo(hstd::Addr_t addr) const;
	bool hasConnInfo(hstd::uid_t uid, hstd::port_t port) const;

	bool addConnection(hstd::Addr_t first, hstd::Addr_t second);
	void removeConnection(hstd::Addr_t first, hstd::Addr_t second);

	void setPartNumOf(const ProxyModule& module);
	void setPartNumOf(hstd::Addr_t addr, enum BOS::module_pn_e part);
	void setPartNumOf(hstd::uid_t id, enum BOS::module_pn_e part);

public:
	void setPortDir(hstd::Addr_t addr, BOS::PortDir dir);
	void setPortDir(hstd::uid_t uid, hstd::port_t port, BOS::PortDir dir);

	void setPortDirNormal(hstd::Addr_t addr);
	void setPortDirReversed(hstd::Addr_t addr);
	void setPortDirNormal(hstd::uid_t uid, hstd::port_t port);
	void setPortDirReversed(hstd::uid_t uid, hstd::port_t port);

	BOS::PortDir getPortDir(hstd::Addr_t addr) const;
	BOS::PortDir getPortDir(hstd::uid_t uid, hstd::port_t port) const;

	bool isPortDirNormal(hstd::Addr_t addr) const;
	bool isPortDirReversed(hstd::Addr_t addr) const;

	bool isPortDirNormal(hstd::uid_t uid, hstd::port_t port) const;
	bool isPortDirReversed(hstd::uid_t uid, hstd::port_t port) const;

public:
	enum BOS::module_pn_e getPartEnumOf(hstd::Addr_t addr) const;
	hstd::Addr_t getModuleConnAt(hstd::Addr_t addr) const;

	hstd::uid_t getUIDConnAt(hstd::Addr_t addr) const;
	hstd::port_t getPortConnAt(hstd::Addr_t addr) const;

	hstd::uid_t getUIDConnAt(hstd::uid_t uid, hstd::port_t port) const;
	hstd::port_t getPortConnAt(hstd::uid_t uid, hstd::port_t port) const;

public:
	ModulesInfo(void);
	~ModulesInfo(void);

private:
	uint16_t array_[BOS::MAX_NUM_OF_MODULES][BOS::MAX_NUM_OF_PORTS + 2];

	static const int PORT_DIR_NDX = 1;
	static const int PART_NUM_NDX = 0;
};

/*********************************************************************************************************/


#endif /* MODULE_INFO_H */