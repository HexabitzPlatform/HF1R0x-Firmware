#ifndef BOSMESSAGE_H
#define BOSMESSAGE_H

#include "helper/BinaryStream.h"
#include "helper/helper.h"
#include "hal/Serial.h"

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

struct Addr_t {

public:
	bool hasValidUID(void) const 		{ return uid <= MAX_UID and uid >= MIN_UID; }
	bool hasValidPort(void) const 		{ return port <= MAX_PORT and port >= MIN_PORT; }
	bool isValid(void) const 			{ return hasValidUID() and hasValidPort(); }

public:
	int getUID(void) const				{ return uid; }
	int getPort(void) const				{ return port; }

	void setUID(int newID) 				{ uid = newID; }
	void setPort(int newPort)			{ port = newPort; }

public:
	Addr_t& operator=(const Addr_t& other) = default;
	Addr_t& operator=(Addr_t&& other) = default;

public:
	Addr_t(void): uid(INVALID_UID), port(INVALID_PORT)				{ }
	Addr_t(int uidIn): uid(uidIn), port(INVALID_PORT)				{ }
	Addr_t(int uidIn, int portIn): uid(uidIn), port(portIn)			{ }
	Addr_t(const Addr_t& other) = default;
	Addr_t(Addr_t&& other) = default;

	~Addr_t(void) = default;

private:		
	int uid;
	int port;

public:
	const static int BROADCAST_UID = 255;
	const static int MULTICAST_UID = 254;
	const static int ACCEPTALL_UID = 0;

private:
	const static int MAX_UID = 255;
	const static int MIN_UID = 0;

	const static int MAX_PORT = 8;
	const static int MIN_PORT = 1;

	const static int INVALID_UID = -1;
	const static int INVALID_PORT = -1;
};


struct Message {
public:
	Addr_t getSource(void) const 		{ return src_; }
	void setSource(Addr_t newSrc) 		{ src_ = newSrc; }

	Addr_t getDest(void) const 			{ return dest_; }
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

		