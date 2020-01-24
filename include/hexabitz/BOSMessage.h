#ifndef BOSMESSAGE_H
#define BOSMESSAGE_H

#include "helper/BinaryStream.h"
#include "helper/helper.h"

#include "hexabitz/BOS.h"
			
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>
#include <ctype.h>

#include <ostream>
#include <iostream>
#include <sstream>
#include <map>



/***********************************************/

// TODO: See if invalidating the object in destructor is good approach

namespace hstd {

typedef int uid_t;
typedef int port_t;


struct Addr_t {
public:
	static bool isValidUID(uid_t id)	{ return id <= MAX_UID and id >= MIN_UID; }
	static bool isUIDed(uid_t id)		{ return id <= MAX_UID and id > MIN_UID; }
	static bool isValidPort(port_t p)	{ return p <= MAX_PORT and p >= MIN_PORT; }
	static bool isMaster(uid_t id)		{ return id == MASTER_UID; }

public:
	bool hasValidUID(void) const 		{ return uid <= MAX_UID and uid >= MIN_UID; }
	bool hasValidPort(void) const 		{ return port <= MAX_PORT and port >= MIN_PORT; }
	bool isValid(void) const 			{ return hasValidUID() and hasValidPort(); }
	bool isMaster(void) const 			{ return uid  == MASTER_UID; }

public:
	uid_t getUID(void) const				{ return uid; }
	port_t getPort(void) const				{ return port; }

	void setUID(uid_t newID) 				{ uid = newID; }
	void setPort(port_t newPort)			{ port = newPort; }

public:
	Addr_t& operator=(const Addr_t& other) = default;
	Addr_t& operator=(Addr_t&& other) = default;

public:
	Addr_t(void): uid(INVALID_UID), port(INVALID_PORT)				{ }
	Addr_t(uid_t uidIn): uid(uidIn), port(INVALID_PORT)				{ }
	Addr_t(uid_t uidIn, port_t portIn): uid(uidIn), port(portIn)			{ }
	Addr_t(const Addr_t& other) = default;
	Addr_t(Addr_t&& other) = default;

	~Addr_t(void) = default;

private:		
	uid_t uid;
	port_t port;

public:
	constexpr static uid_t ACCEPTALL_UID = 0;
	constexpr static uid_t MASTER_UID = 1;
	constexpr static uid_t MULTICAST_UID = 254;
	constexpr static uid_t BROADCAST_UID = 255;

public:
	constexpr static uid_t MAX_UID = 255;
	constexpr static uid_t MIN_UID = 0;

	constexpr static port_t MAX_PORT = BOS::MAX_NUM_OF_PORTS;
	constexpr static port_t MIN_PORT = 1;

	constexpr static uid_t INVALID_UID = -1;
	constexpr static port_t INVALID_PORT = -1;
};


struct Message {
public:
	const Addr_t& getSource(void) const { return src_; }
	Addr_t& getSource(void)		 		{ return src_; }
	void setSource(Addr_t newSrc) 		{ src_ = newSrc; }

	const Addr_t& getDest(void) const 	{ return dest_; }
	Addr_t& getDest(void)	 			{ return dest_; }
	void setDest(Addr_t newDest) 		{ dest_ = newDest; }

	uint16_t getCode(void) const 		{ return code_; }
	void setCode(uint16_t newCode) 		{ code_ = newCode; }

	void setMessOnlyFlag(bool flag) 	{ setFlag("messonly", flag); }
	bool getMessOnlyFlag(void) 			{ return getFlag("messonly"); }

	void setCLIOnlyFlag(bool flag) 		{ setFlag("clionly", flag); }
	bool getCLIOnlyFlag(void) 			{ return getFlag("clionly"); }

	void setTraceFlag(bool flag) 		{ setFlag("trace", flag); }
	bool getTraceFlag(void) 			{ return getFlag("trace"); }

	BinaryBuffer& getParams(void) 				{ return param_; }
	const BinaryBuffer& getParams(void) const 	{ return param_; }

public:
	void setFlag(std::string name, bool value);
	bool getFlag(std::string name);

public:
	void invalidate(void);

public:
	friend std::ostream& operator<<(std::ostream& stream, Message& m)
	{
		stream << std::string(m) << std::endl;
		return stream;
	}

	operator std::string(void)
	{
		return to_string();
	}

	std::string to_string(void) const;

public:
	Message& operator=(const Message& other) = default;
	Message& operator=(Message&& other) = default;

public:
	Message(void);
	Message(Addr_t dest, Addr_t src, uint16_t code);
	Message(uint8_t dest, uint8_t src, uint16_t code);
	Message(const Message& other) = default;
	Message(Message&& other) = default;

	~Message(void) = default;

private:
	Addr_t src_;
	Addr_t dest_;
	uint16_t code_;
	BinaryBuffer param_;

	std::map<std::string, bool> flags_;
};

/***********************************************/



bool parseMessage(BinaryBuffer buffer, Message& msg);
std::vector<Message> getSanitizedMessages(Message msg);
Message buildMessage(uint8_t dst, uint8_t src, BinaryBuffer buffer);

}


#endif /* BOSMESSAGE_H */

		