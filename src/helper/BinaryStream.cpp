#include "helper/BinaryStream.h"
#include "helper/helper.h"

#include <math.h>
#include <endian.h>




BinaryStream::BinaryStream(void): data_(), readNdx_(0)
{

}

BinaryStream::~BinaryStream(void)
{

}

size_t BinaryStream::append(uint8_t ui8)
{
	data_.push_back(ui8);
	return data_.size();
}

size_t BinaryStream::append(uint16_t ui16)
{
	ui16 = htole16(ui16);
	return append(reinterpret_cast<uint8_t *>(ui16), sizeof(ui16));
}

size_t BinaryStream::append(uint32_t ui32)
{
	ui32 = htole32(ui32);
	return append(reinterpret_cast<uint8_t *>(ui32), sizeof(ui32));
}

size_t BinaryStream::append(float f)
{
	uint32_t value = htole32(hstd::pack754(f, 32, 8));
	return append(value);
}


size_t BinaryStream::append(const uint8_t *array, size_t num)
{
	for (size_t i = 0; i < num; i++)
		data_.push_back(array[i]);
	return data_.size();
}

size_t BinaryStream::append(const std::vector<uint8_t>& v)
{
	data_.insert(data_.end(), v.begin(), v.end());
	return data_.size();
}

uint8_t BinaryStream::getValue(size_t ndx) const
{
	if (ndx >= data_.size())
		return 0;
	return data_[ndx];
}

uint8_t BinaryStream::popui8(void)
{
	if (readNdx_ >= data_.size())
		return 0;
	return data_[readNdx_++];
}

uint16_t BinaryStream::popui16(void)
{
	uint16_t value;
	const int sizeOfValue = sizeof(value);
	if ((readNdx_ + sizeOfValue) >=	 data_.size())
		return 0;
	
	std::copy(data_.begin() + readNdx_, data_.begin() + readNdx_ + sizeOfValue, reinterpret_cast<uint8_t *>(value));
	readNdx_ += sizeOfValue;
	return le16toh(value);
}

uint32_t BinaryStream::popui32(void)
{
	uint16_t value;
	const int sizeOfValue = sizeof(value);
	if ((readNdx_ + sizeOfValue) >=	 data_.size())
		return 0;
	
	std::copy(data_.begin() + readNdx_, data_.begin() + readNdx_ + sizeOfValue, reinterpret_cast<uint8_t *>(value));
	readNdx_ += sizeOfValue;
	return le32toh(value);
}

float BinaryStream::popfloat(void)
{
	uint32_t value = popui32();
	return hstd::unpack754(value, 32, 8);
}