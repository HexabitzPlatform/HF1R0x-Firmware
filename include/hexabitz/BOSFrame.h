#ifndef BOSFRAME_H
#define BOSFRAME_H

#include "helper/BinaryStream.h"
#include "hexabitz/BOSMessage.h"
#include "helper/helper.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdarg.h>
#include <ctype.h>

#include <ostream>

namespace hstd {

struct Frame {
public:
	uint8_t getTotalLength(void) const 				{ return param.getLength() + getMinLength(); }

	static uint8_t getMinLength(void) 				{ return MIN_LENGTH; }
	static uint8_t getMaxLength(void) 				{ return MAX_LENGTH; }
	static uint8_t getMaxParamLength(void) { return getMaxLength() - getMinLength(); }

public:
	void setCodeOnly(uint16_t newCode);
	uint16_t getCodeOnly(void) const;

	void setFlag(int bit, bool value) 	{ bitWrite(code, bit, value); }
	bool getFlag(int bit) const			{ return bitRead(code, bit); }

	bool isLong(void) const 	{ return bitRead(code, LONG_FLAG_BITPOS); }

public:
	void sanitize(void);
	bool isValid(void) const;

public:
	bool fromBuffer(BinaryBuffer& buffer);
	BinaryBuffer toBuffer(void) const;

public:
	Frame& operator=(const Frame& other) = default;
	Frame& operator=(Frame&& other) = default;

public:
	Frame(void);
	Frame(const Frame& other) = default;
	Frame(Frame&& other) = default;
	
	~Frame(void) = default;

public:
	uint8_t destID;
	uint8_t srcID;
	uint16_t code;
	BinaryBuffer param;
	uint8_t crc8;

public:
	static const uint16_t CODE_ONLY_MASK = 0x8FFF;
	static const uint8_t CRC8_FIXED = 0x75;

	static const uint8_t MIN_LENGTH = 5;
	static const uint8_t MAX_LENGTH = 50;

	static const int TRACE_FLAG_BITPOS = 12;
	static const int CLI_FLAG_BITPOS = 13;
	static const int MESS_FLAG_BITPOS = 14;
	static const int LONG_FLAG_BITPOS = 15;
	
};

std::vector<Frame> buildFramesFromMessage(Message message);
bool buildMessageFromFrames(std::vector<Frame>& vec, Message& message);

}	// namespace hstd



#endif	/* BOSFRAME_H */