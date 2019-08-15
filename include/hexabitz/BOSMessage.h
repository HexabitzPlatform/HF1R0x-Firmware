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

#include <endian.h>


/***********************************************/

namespace hstd {

// TODO: Add struct for code_
struct Message {

public:
	uint8_t getSource(void) const { return src_; }
	void setSource(uint8_t newSrc) { src_ = newSrc; }

	uint8_t getDest(void) const { return dest_; }
	void setDest(uint8_t newDest) { dest_ = newDest; }

	uint16_t getCode(void) const { return code_; }
	void setCode(uint16_t newCode) { code_ = newCode; }

	void setLongFlag(bool flag) { bitWrite(code_, 15, flag); }
	bool getLongFlag(void) { return bitRead(code_, 15); }

	void setMessOnlyFlag(bool flag) { bitWrite(code_, 14, flag); }
	bool getMessOnlyFlag(void) { return bitRead(code_, 14); }

	void setCLIOnlyFlag(bool flag) { bitWrite(code_, 13, flag); }
	bool getCLIOnlyFlag(void) { return bitRead(code_, 13); }

	void setTraceFlag(bool flag) { bitWrite(code_, 12, flag); }
	bool getTraceFlag(void) { return bitRead(code_, 12); }

	BinaryBuffer& getParams(void) { return param_; }

	uint8_t getTotalLength(void) const { return param_.getLength() + getMinLength(); }
	static uint8_t getMinLength(void) { return sizeof(src_) + sizeof(dest_) + sizeof(code_) + sizeof(crc8_); }
	static uint8_t getParamLength(uint8_t totalLen) { return totalLen - getMinLength(); }


	bool parse(BinaryBuffer buffer)
	{
		if (buffer.getLength() < getMinLength())
			return false;

		uint8_t len = buffer.popui8();
		if (len != buffer.getLength())
			return false;

		dest_ = buffer.popui8();
		src_ = buffer.popui8();
		code_ = buffer.popui16();
		param_.append(buffer, getParamLength(len));
		crc8_ = buffer.popui8();

		return true;
	}

	void sanitize(void)
	{
		if (getTotalLength() == 13)
			param_.append(uint8_t(0));

		// TODO: Calculate CRC
		crc8_ = 0x75; // Fixed Value of CRC
	}

	bool validate(void) const
	{
		// Check againt mimumum size
		if (getTotalLength() == 13)
			return false;

		uint8_t calCRC = 0x75;
		if (calCRC != crc8_)
			return false;

		return true;
	}
 
	BinaryBuffer getBinaryStream(void) const
	{
		BinaryBuffer buffer;
		BinaryBuffer temp(param_);
		buffer.append(getTotalLength());
		buffer.append(dest_);
		buffer.append(src_);
		buffer.append(code_);
		buffer.append(temp);
		buffer.append(crc8_);
		return buffer;
	}

	friend std::ostream& operator<<(std::ostream& stream, Message& m)
	{
		BinaryBuffer buffer = m.getBinaryStream();
		stream << "Packet: " << buffer.getLength() << " (";
		for (int i = 0; i < buffer.getLength(); i++)
			stream << std::hex << "0x" << int(buffer[i]) << " ";

		stream << ")" << std::endl;
		return stream;
	}

	operator std::string(void)
	{
		std::stringstream stream;
		stream << ">>> Header (Len: " << std::hex << int(getTotalLength()) << " | ";
		stream << "Destination: 0x"  << std::hex << int(getDest()) << " | ";
		stream << "Source: 0x" << std::hex << int(getSource()) << " | ";
		stream << "Code: 0x"  << std::hex << int(getCode()) << ")" << std::endl;

		stream << "Parameters: ";
		for (int i = 0; (i < param_.getLength()); i++)
			stream << std::hex << "0x" << int(param_[i]) << " ";
		if (!param_.getLength()) 
			stream << "--";
		stream  << std::endl;

		stream << "CRC: 0x" << std::hex << int(crc8_) << " <<<" << std::endl;

		return stream.str();
	}


private:
	uint8_t src_;
	uint8_t dest_;
	uint16_t code_;
	BinaryBuffer param_;
	uint8_t crc8_;
};
/***********************************************/



Message read(HardwareSerial& serial_);
bool write(HardwareSerial& serial_, Message& m);

}


#endif /* BOSMESSAGE_H */

		