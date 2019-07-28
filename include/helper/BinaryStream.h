#ifndef BINARYSTREAM_H
#define BINARYSTREAM_H

#include "hal/Serial.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include <vector>


class BinaryStream {

public:
	size_t getLength(void) const
	{
		return data_.size();
	}

	void setReadNdx(size_t newNdx)
	{
		readNdx_ = newNdx;
	}
	void resetNdx(void)
	{
		readNdx_ = 0;
	}
	void resetData(void)
	{
		data_.clear();
		readNdx_ = 0;
	}

public:
	size_t append(uint8_t ui8);
	size_t append(uint16_t ui16);
	size_t append(uint32_t ui32);
	size_t append(float f);
	size_t append(const uint8_t *array, size_t num);
	size_t append(const std::vector<uint8_t>& v);

public:
	uint8_t popui8(void);
	uint16_t popui16(void);
	uint32_t popui32(void);
	float popfloat(void);

	uint8_t getValue(size_t ndx) const;
	const std::vector<uint8_t>& getVector(void) const;

public:
	BinaryStream(void);
	~BinaryStream(void);

private:
	std::vector<uint8_t> data_;
	std::size_t readNdx_;
};




#endif	// BINARYSTREAM_H