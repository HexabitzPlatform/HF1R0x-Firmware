#include "hexabitz/ModuleInfo.h"

#include "hexabitz/ProxyModule.h"

#include <string.h>


/***************************************** Neighbours Info Class *****************************************/

NeighboursInfo::NeighboursInfo(void)
{
	memset(array_, 0, sizeof(array_));
}

NeighboursInfo::~NeighboursInfo(void)
{

}

BinaryBuffer NeighboursInfo::toBinaryBuffer(void) const
{
	BinaryBuffer buffer;
	for (int i = 0; i < BOS::MAX_NUM_OF_PORTS; i++) {
		buffer.append(uint16_t(array_[i][ADDR_NDX]));
		buffer.append(uint16_t(array_[i][PART_NDX]));
	}
	return buffer;
}

void NeighboursInfo::fromBinaryBuffer(BinaryBuffer buffer)
{
	memset(array_, 0, sizeof(array_));
	
	for (int i = 0; i < BOS::MAX_NUM_OF_PORTS; i++) {
		array_[i][ADDR_NDX] = buffer.popui16();
		array_[i][PART_NDX] = buffer.popui16();
	}
}

void NeighboursInfo::reset(void)
{
	memset(array_, 0, sizeof(array_));
}

bool NeighboursInfo::hasInfo(hstd::port_t port) const
{
	return !!(array_[port - 1][ADDR_NDX]);
}

bool NeighboursInfo::hasIDedInfo(hstd::port_t port) const
{
	return hstd::Addr_t::isValidPort(getPortAt(port)) and hstd::Addr_t::isUIDed(getUIDAt(port));
}

bool NeighboursInfo::hasUnIDedInfo(hstd::port_t port) const
{
	return !hasIDedInfo(port);
}

bool NeighboursInfo::hasAllIDedInfo(void) const
{
	for (hstd::port_t p = 1 ; p <= BOS::MAX_NUM_OF_PORTS; p++) {
		if (hasUnIDedInfo(p))
			return false;
	}
	return true;
}

void NeighboursInfo::setAddrInfoFor(hstd::port_t port, hstd::Addr_t addr)
{
	uint16_t ui16UID = addr.getUID();
	uint16_t ui16Port = addr.getPort();
	array_[port - 1][ADDR_NDX] = (ui16UID << 8) + (ui16Port & 0x00FF);
}

void NeighboursInfo::setUIDInfoFor(hstd::port_t port, hstd::uid_t uid)
{
	uint16_t ui16UID = uid;
	array_[port - 1][ADDR_NDX] = (ui16UID << 8) + (array_[port - 1][ADDR_NDX] & 0x00FF);
}

void NeighboursInfo::setPortInfoFor(hstd::port_t port, hstd::port_t neighPort)
{
	uint16_t ui16Port = neighPort;
	array_[port - 1][ADDR_NDX] = (array_[port - 1][ADDR_NDX] & 0xFF00) + (ui16Port & 0x00FF);
}

void NeighboursInfo::setPartInfoFor(hstd::port_t port, enum BOS::module_pn_e part)
{
	uint16_t ui16Part = static_cast<uint16_t>(part);
	array_[port - 1][PART_NDX] = ui16Part;
}

enum BOS::module_pn_e NeighboursInfo::getPartEnumAt(hstd::port_t port) const
{
	return static_cast<enum BOS::module_pn_e>(array_[port - 1][PART_NDX]);
}

hstd::Addr_t NeighboursInfo::getAddrAt(hstd::port_t port) const
{
	return hstd::Addr_t(getUIDAt(port), getPortAt(port));
}

hstd::uid_t NeighboursInfo::getUIDAt(hstd::port_t port) const
{
	return hstd::uid_t(highByte(array_[port - 1][ADDR_NDX]));
}

hstd::port_t NeighboursInfo::getPortAt(hstd::port_t port) const
{
	return hstd::port_t(lowByte(array_[port -1][ADDR_NDX]));
}

/*********************************************************************************************************/



/******************************************* Module Info Class *******************************************/

ModulesInfo::ModulesInfo(void)
{
	memset(array_, 0, sizeof(array_));
}

ModulesInfo::~ModulesInfo(void)
{

}

BinaryBuffer ModulesInfo::toBinaryBuffer(int num) const
{
	BinaryBuffer buffer;
	if (num < 0)
		num = BOS::MAX_NUM_OF_MODULES;
	for (int i = 0; i < num; i++) {
		buffer.append(uint16_t(array_[i][PART_NUM_NDX]));

		for (int j = 0; j < BOS::MAX_NUM_OF_PORTS; j++)
			buffer.append(uint16_t(array_[i][j + 2]));
	}
	return buffer;
}

void ModulesInfo::fromBinaryBuffer(BinaryBuffer buffer)
{
	memset(array_, 0, sizeof(array_));
	
	for (int i = 0; i < BOS::MAX_NUM_OF_MODULES; i++) {
		array_[i][PART_NUM_NDX] = buffer.popui16();
		array_[i][PORT_DIR_NDX] = 0;

		for (int j = 0; j < BOS::MAX_NUM_OF_PORTS; j++)
			array_[i][j + 2] = buffer.popui16();
	}
}

void ModulesInfo::reset(void)
{
	memset(array_, 0, sizeof(array_));
}

bool ModulesInfo::hasConnInfo(hstd::Addr_t addr) const
{
	return addr.isValid() and (array_[addr.getUID() - 1][addr.getPort() + 1] != 0);
}

bool ModulesInfo::hasConnInfo(hstd::uid_t uid, hstd::port_t port) const
{
	return hasConnInfo(hstd::Addr_t(uid, port));
}

bool ModulesInfo::addConnection(hstd::Addr_t first, hstd::Addr_t second)
{
	uint16_t ui16FirstUID = first.getUID();
	uint16_t ui16FirstPort = first.getPort();

	uint16_t ui16SecondUID = second.getUID();
	uint16_t ui16SecondPort = second.getPort();

	array_[ui16FirstUID - 1][ui16FirstPort + 1] = (ui16SecondUID << 3 )	 | (ui16SecondPort & 0b111);	/* Neighbor ID | Neighbor port */
	array_[ui16SecondUID - 1][ui16SecondPort + 1] = (ui16FirstUID << 3 ) | (ui16FirstPort & 0b111);	/* Module 1 ID | Module 1 port */
	return true;
}

void ModulesInfo::removeConnection(hstd::Addr_t first, hstd::Addr_t second)
{
	array_[first.getUID() - 1][first.getPort() + 1] = 0;
	array_[second.getUID() - 1][second.getPort() + 1] = 0;
}

void ModulesInfo::setPartNumOf(const ProxyModule& module)
{
	setPartNumOf(module.getUID(), module.getPartNum());
}

void ModulesInfo::setPartNumOf(hstd::Addr_t addr, enum BOS::module_pn_e part)
{
	array_[addr.getUID() - 1][PART_NUM_NDX] = static_cast<uint16_t>(part);
}

void ModulesInfo::setPartNumOf(hstd::uid_t id, enum BOS::module_pn_e part)
{
	array_[id - 1][PART_NUM_NDX] = static_cast<uint16_t>(part);
}

void ModulesInfo::setPortDir(hstd::Addr_t addr, BOS::PortDir dir)
{
	setPortDir(addr.getUID(), addr.getPort(), dir);
}

void ModulesInfo::setPortDir(hstd::uid_t uid, hstd::port_t port, BOS::PortDir dir)
{
	if (dir == BOS::PortDir::REVERSED)
		array_[uid - 1][PORT_DIR_NDX] |= (0x8000 >> (port - 1));
	else
		array_[uid - 1][PORT_DIR_NDX] &= (~(0x8000 >> (port - 1)));	
}

void ModulesInfo::setPortDirNormal(hstd::Addr_t addr)
{
	return setPortDir(addr, BOS::PortDir::NORMAL);
}

void ModulesInfo::setPortDirReversed(hstd::Addr_t addr)
{
	return setPortDir(addr, BOS::PortDir::REVERSED);
}

void ModulesInfo::setPortDirNormal(hstd::uid_t uid, hstd::port_t port)
{
	return setPortDir(uid, port, BOS::PortDir::NORMAL);
}

void ModulesInfo::setPortDirReversed(hstd::uid_t uid, hstd::port_t port)
{
	return setPortDir(uid, port, BOS::PortDir::REVERSED);
}

BOS::PortDir ModulesInfo::getPortDir(hstd::Addr_t addr) const
{
	hstd::uid_t uid = addr.getUID();
	hstd::port_t port = addr.getPort();

	if (array_[uid - 1][PORT_DIR_NDX] & (0x8000 >> (port - 1)))
		return BOS::PortDir::REVERSED;
	return BOS::PortDir::NORMAL;
}

BOS::PortDir ModulesInfo::getPortDir(hstd::uid_t uid, hstd::port_t port) const
{
	return getPortDir(hstd::Addr_t(uid, port));
}

bool ModulesInfo::isPortDirNormal(hstd::Addr_t addr) const
{
	return getPortDir(addr) == BOS::PortDir::NORMAL;
}

bool ModulesInfo::isPortDirReversed(hstd::Addr_t addr) const
{
	return getPortDir(addr) == BOS::PortDir::REVERSED;
}

bool ModulesInfo::isPortDirNormal(hstd::uid_t uid, hstd::port_t port) const
{
	return isPortDirNormal(hstd::Addr_t(uid, port));
}

bool ModulesInfo::isPortDirReversed(hstd::uid_t uid, hstd::port_t port) const
{
	return isPortDirReversed(hstd::Addr_t(uid, port));
}

enum BOS::module_pn_e ModulesInfo::getPartEnumOf(hstd::Addr_t addr) const
{
	return static_cast<enum BOS::module_pn_e>(array_[addr.getUID() - 1][PART_NUM_NDX]);
}

hstd::Addr_t ModulesInfo::getModuleConnAt(hstd::Addr_t addr) const
{
	return hstd::Addr_t(getUIDConnAt(addr), getPortConnAt(addr));
}

hstd::uid_t ModulesInfo::getUIDConnAt(hstd::Addr_t addr) const
{
	return hstd::uid_t(array_[addr.getUID() - 1][addr.getPort() + 1] >> 3);
}

hstd::port_t ModulesInfo::getPortConnAt(hstd::Addr_t addr) const
{
	return hstd::port_t(array_[addr.getUID() - 1][addr.getPort() + 1] & 0b111);
}

hstd::uid_t ModulesInfo::getUIDConnAt(hstd::uid_t uid, hstd::port_t port) const
{
	return getUIDConnAt(hstd::Addr_t(uid, port));
}

hstd::port_t ModulesInfo::getPortConnAt(hstd::uid_t uid, hstd::port_t port) const
{
	return getPortConnAt(hstd::Addr_t(uid, port));
}

/*********************************************************************************************************/