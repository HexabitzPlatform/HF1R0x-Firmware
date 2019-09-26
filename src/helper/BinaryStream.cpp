#include "helper/BinaryStream.h"
#include "helper/helper.h"

#include <fstream>


BinaryBuffer::BinaryBuffer(void): data_()	
{

}

BinaryBuffer::~BinaryBuffer(void)
{

}

bool BinaryBuffer::save(std::string filename) const
{
	std::ofstream stream(filename, std::ios::binary);
	if (!stream.is_open())
		return false;

	for (const char d: data_)
		stream.write(&d, 1);

	return true;
}

bool BinaryBuffer::restore(std::string filename)
{
	std::ifstream stream(filename.c_str(), std::ios::binary);
	if (!stream.is_open())
		return false;

	char d;
	data_.clear();
	while (!stream.eof()) {
		stream.read(&d, 1);
		data_.push_back(uint8_t(d));
	}
	return true;
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
