#include "hexabitz/BOSMessageBuilder.h"


static bool cliFlag_ = false;
static bool messFlag_ = false;
static bool traceFlag_ = false;



void hstd::setCLIRespDefault(bool enable)
{
	if (enable)
		setMessRespDefault(true);
	
	cliFlag_ = enable;
}

void hstd::setMessRespDefault(bool enable)
{
	messFlag_ = enable;
}

void hstd::setTraceDefault(bool enable)
{
	traceFlag_ = enable;
}

hstd::Message hstd::make_message(Addr_t dest, uint16_t code)
{
	return make_message(dest, Addr_t(), code);
}

hstd::Message hstd::make_message(Addr_t dest, Addr_t src, uint16_t code)
{
	hstd::Message message;
	
	message.setSource(src);
	message.setDest(dest);
	message.setCode(code);

	message.setCLIOnlyFlag(cliFlag_);
	message.setMessOnlyFlag(messFlag_);
	message.setTraceFlag(traceFlag_);

	return message;
}

hstd::Message hstd::make_ping_message(Addr_t dest, Addr_t src)
{
	return make_message(dest, src, CODE_ping);
}

hstd::Message hstd::make_ping_message(hstd::uid_t destID, hstd::uid_t srcID)
{
	return make_message(destID, srcID, CODE_ping);
}

hstd::Message hstd::make_ping_message(hstd::uid_t destID)
{
	return make_message(destID, CODE_ping);
}

hstd::Message hstd::make_message(hstd::uid_t destID, uint16_t code)
{
	return hstd::make_message(hstd::Addr_t(destID), code);
}

hstd::Message hstd::make_message(hstd::uid_t destID, hstd::uid_t srcID, uint16_t code)
{
	return hstd::make_message(hstd::Addr_t(destID), hstd::Addr_t(srcID), code);
}

hstd::Message hstd::make_broadcast(uint16_t code)
{
	return hstd::make_message(hstd::Addr_t::BROADCAST_UID, code);
}

hstd::Message hstd::make_message_meighbour(hstd::port_t portNum, uint16_t code)
{
	return hstd::make_message(hstd::Addr_t(0), hstd::Addr_t(0, portNum), code);
}