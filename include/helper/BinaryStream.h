#ifndef BINARYSTREAM_H
#define BINARYSTREAM_H

#include "hal/Serial.h"
#include "helper/GenericStream.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include <vector>


class BinaryBuffer: virtual public IBinaryStream, virtual public OBinaryStream {

public:
	size_t getLength(void) const
	{
		return data_.size();
	}
	void flush(void)
	{

	}

	void reset(void)
	{
		data_.clear();
	}

	uint8_t& operator[](int ndx)
	{
		return data_[ndx];
	}

	const uint8_t& operator[](int ndx) const
	{
		return data_[ndx];
	}

public:
	using IBinaryStream::append;
	
	IBinaryStream& append(uint8_t ui8);
	uint8_t popui8(void);

public:
	BinaryBuffer(void);
	~BinaryBuffer(void);

private:
	std::vector<uint8_t> data_;
};




#endif	// BINARYSTREAM_H