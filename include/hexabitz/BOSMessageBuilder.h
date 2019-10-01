#ifndef BOS_MESSAGE_BUILDER_H
#define BOS_MESSAGE_BUILDER_H

#include "hexabitz/BOS.h"
#include "hexabitz/BOSMessage.h"

#include <stdint.h>
#include <stdbool.h>

namespace hstd {

void setCLIRespDefault(bool enable);
void setMessRespDefault(bool enable);
void setTraceDefault(bool enable);

Message make_message(Addr_t dest, uint16_t code);
Message make_message(Addr_t dest, Addr_t src, uint16_t code);

Message make_ping_message(Addr_t dest, Addr_t src);
Message make_ping_message(uid_t destID, uid_t srcID);
Message make_ping_message(uid_t destID);

Message make_message(uid_t destId, uint16_t code);
Message make_message(uid_t destId, uid_t srcID, uint16_t code);

Message make_message_meighbour(port_t portNum, uint16_t code);

Message make_broadcast(uint16_t code);
}


#endif /* BOS_MESSAGE_BUILDER_H */