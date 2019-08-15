#include "helper/BinaryStream.h"
#include "helper/helper.h"



BinaryBuffer::BinaryBuffer(void): data_()	
{

}

BinaryBuffer::~BinaryBuffer(void)
{

}

IBinaryStream& BinaryBuffer::append(uint8_t ui8)
{
	data_.push_back(ui8);
	return *this;
}


uint8_t BinaryBuffer::popui8(void)
{
	if (!data_.size())
		return 0;

	uint8_t value = data_.front();
	data_.erase(data_.begin());
	return value;
}
